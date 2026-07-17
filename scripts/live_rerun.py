#!/usr/bin/env python3
"""
live_rerun.py — interactive 3D map + LIVE robot pose in Rerun (viewer side).

Runs on the viewing machine (desktop / laptop), NOT on the Pi — all rendering
AND message decoding happen on the viewer.

Two transports (--transport):
  foxglove (default): connects to foxglove_bridge (:8765, C++). The bridge
      forwards raw CDR binary without re-encoding, so the Pi pays ~zero cost
      and full-rate TF/video stay smooth; this client decodes CDR locally
      (rosbags). Use this one.
  rosbridge: legacy path via rosbridge_websocket (:9090, Python). Each
      message is JSON-encoded (images base64) ON THE PI, which competes with
      the SLAM stack for CPU and lags under load. Kept as fallback.

Shows, all in the SLAM `map` frame:
  - offline Gaussian-splat model      (--splat splat.ply, rendered as colored
                                       points; see docs/gaussian_splatting.md
                                       for full-quality splat viewers)
  - and/or the RTAB-Map cloud         (--cloud rtabmap_cloud.ply)
  - live robot pose from TF           (map -> odom -> base_link -> camera...)
  - camera FOV frustum + live video   (rr.Pinhole + compressed JPEG)
  - growing trajectory polyline

For this to be meaningful after a reboot, put the robot in `loc_3d` (or
`slam_3d`) mode first so rtabmap publishes map->odom; in `idle`/2D modes the
robot is only shown in the odom frame glued to map origin.

Phone / tablet: add --serve and open http://<this-pc>:9090/?url=... (printed
at startup) — the Rerun web viewer renders in the phone's browser.

Dependencies (viewer machine only, no ROS needed):
    pip install rerun-sdk numpy plyfile websockets rosbags   # foxglove path
    pip install roslibpy                                     # rosbridge path

Usage:
    python3 live_rerun.py --robot 192.168.1.42 --splat splat.ply
    python3 live_rerun.py --robot mentorpi.local --cloud rtabmap_maps/rtabmap_cloud.ply --serve
"""

import argparse
import base64
import io
import json
import math
import re
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rerun as rr

MAP = "map"  # rerun root entity == ROS map frame

# rerun >= 0.23 unified the time API into rr.set_time(); older SDKs
# (e.g. 0.22) only have set_time_seconds. Pick once at import.
if hasattr(rr, "set_time"):
    def set_ros_time(seconds: float) -> None:
        rr.set_time("ros_time", timestamp=seconds)
else:  # rerun < 0.23
    def set_ros_time(seconds: float) -> None:
        rr.set_time_seconds("ros_time", seconds)


def quat_from_rpy(r: float, p: float, y: float) -> np.ndarray:
    """ROS fixed-axis RPY -> xyzw quaternion."""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


# ---------- ply loading (normal colored ply OR 3DGS ply) ----------

SH_C0 = 0.28209479177387814


def load_ply_points(path: Path):
    """Return (xyz, rgb) from a regular colored PLY or a 3DGS splat PLY."""
    from plyfile import PlyData
    ply = PlyData.read(str(path))
    v = ply["vertex"].data
    names = set(v.dtype.names)
    xyz = np.stack([v["x"], v["y"], v["z"]], axis=1).astype(np.float32)

    if {"red", "green", "blue"}.issubset(names):
        rgb = np.stack([v["red"], v["green"], v["blue"]], axis=1).astype(np.uint8)
    elif {"f_dc_0", "f_dc_1", "f_dc_2"}.issubset(names):
        # Gaussian-splat PLY: colors are 0th-order spherical harmonics
        dc = np.stack([v["f_dc_0"], v["f_dc_1"], v["f_dc_2"]], axis=1)
        rgb = np.clip((0.5 + SH_C0 * dc) * 255.0, 0, 255).astype(np.uint8)
    else:
        rgb = None

    # Drop nearly-transparent splats — they are noise for a point rendering.
    if "opacity" in names:
        alpha = 1.0 / (1.0 + np.exp(-np.asarray(v["opacity"])))
        keep = alpha > 0.3
        xyz = xyz[keep]
        if rgb is not None:
            rgb = rgb[keep]
    return xyz, rgb


# ---------- RGB-D backprojection (true-color live depth cloud) ----------

def decode_jpeg_rgb(data: bytes) -> Optional[np.ndarray]:
    """JPEG/PNG bytes -> HxWx3 uint8, or None if no decoder available."""
    try:
        from PIL import Image as PILImage
    except ImportError:
        return None
    try:
        return np.asarray(PILImage.open(io.BytesIO(data)).convert("RGB"))
    except Exception:
        return None


def backproject_depth(depth_mm: np.ndarray, K: np.ndarray,
                      rgb: Optional[np.ndarray], stride: int,
                      max_depth: float):
    """16UC1 depth (mm, optical frame, aligned 1:1 with rgb) -> (Nx3 points
    in the optical frame, Nx3 uint8 colors or None). Gemini 2L publishes
    HW-aligned depth with the color K and color frame, so pixel (u,v) in
    depth IS pixel (u,v) in rgb — no remapping needed."""
    h, w = depth_mm.shape
    d = depth_mm[::stride, ::stride].astype(np.float32) / 1000.0
    uu, vv = np.meshgrid(np.arange(0, w, stride, dtype=np.float32),
                         np.arange(0, h, stride, dtype=np.float32))
    valid = (d > 0.1) & (d < max_depth)
    z = d[valid]
    x = (uu[valid] - K[0, 2]) * z / K[0, 0]
    y = (vv[valid] - K[1, 2]) * z / K[1, 1]
    pts = np.stack([x, y, z], axis=1)
    colors = None
    if rgb is not None and rgb.shape[:2] == depth_mm.shape:
        colors = rgb[::stride, ::stride][valid]
    return pts, colors


# ---------- TF tree ----------

class TfTree:
    """Keeps latest transform per edge and mirrors frames into Rerun.

    Every TF frame lives at a STABLE entity path `map/<frame>` carrying its
    composed map<-frame ABSOLUTE pose. An earlier design nested paths along
    the TF chain (map/odom/base_link/...); when the topology filled in
    (rtabmap's map->odom appearing after startup) entities MOVED to new
    paths, stranding statically-logged Pinholes/images at the old ones —
    the "frozen extra frustum + duplicate RGB panel" bug. Flat paths never
    move, so nothing can go stale. On each edge update we re-log the frame
    and all its TF descendants (~10 frames, cheap).
    """

    def __init__(self) -> None:
        self.parent_of: dict[str, str] = {}
        # child -> (translation, quaternion, is_static)
        self.xform: dict[str, tuple[np.ndarray, np.ndarray, bool]] = {}
        self.lock = threading.Lock()

    @staticmethod
    def path(frame: str) -> str:
        """Stable entity path for a TF frame (never changes, never None)."""
        return MAP if frame == MAP else f"{MAP}/{frame}"

    def update(self, child: str, parent: str, t: np.ndarray, q: np.ndarray,
               *, static: bool, stamp: float) -> None:
        to_log: list[tuple[str, np.ndarray, np.ndarray, bool]] = []
        with self.lock:
            self.parent_of[child] = parent
            self.xform[child] = (t, q, static)
            # The absolute pose of `child` AND every frame hanging under it
            # changed; recompute and re-log them all.
            affected = [child]
            i = 0
            while i < len(affected):
                cur = affected[i]
                i += 1
                affected.extend(c for c, p in self.parent_of.items() if p == cur)
            for f in affected:
                M = self._map_from_locked(f)
                if M is None:
                    continue
                # Absolute poses are always temporal: even a /tf_static edge
                # (base_link->camera_link) moves in the map frame whenever an
                # ancestor moves.
                to_log.append((self.path(f), M[:3, 3].copy(),
                               rot_to_quat(M[:3, :3]), False))
        if not static:
            set_ros_time(stamp)
        for p, ct, cq, _cs in to_log:
            rr.log(p, rr.Transform3D(translation=ct,
                                     rotation=rr.Quaternion(xyzw=cq)))

    def _map_from_locked(self, frame: str) -> Optional[np.ndarray]:
        M = np.eye(4)
        cur = frame
        seen = set()
        while cur != MAP:
            if cur in seen:
                return None  # cycle
            seen.add(cur)
            edge = self.xform.get(cur)
            parent = self.parent_of.get(cur)
            if edge is None or parent is None:
                break  # orphan root == map origin
            t, q, _ = edge
            E = np.eye(4)
            E[:3, :3] = quat_to_rot(q)
            E[:3, 3] = t
            M = E @ M
            cur = parent
        return M

    def map_from(self, frame: str) -> Optional[np.ndarray]:
        """Compose map<-frame as a 4x4 by walking stored edges. An orphan
        chain root is treated as sitting at the map origin (consistent with
        the entity paths above)."""
        with self.lock:
            return self._map_from_locked(frame)


class RobotModel:
    """Visual robot model: STL meshes hung under the base_link entity.

    Geometry (calibrated wheel offsets, laser/camera joint origins) is
    parsed straight out of mecanum.xacro so the viewer stays in sync with
    calibration write-backs — no hardcoded copies. Entity paths are stable
    (map/base_link), so ensure() logs the meshes once and no-ops after;
    the move-handling branch is kept as a safety net. Fail-soft: if the
    repo layout / meshes are missing, the viewer still works, just without
    the robot body.
    """

    def __init__(self) -> None:
        self.base_path: Optional[str] = None
        try:
            self.parts = self._load()
        except Exception as e:  # missing xacro/meshes: degrade gracefully
            print(f"robot model unavailable ({e}); showing axes only")
            self.parts = []

    @staticmethod
    def _load():
        root = Path(__file__).resolve().parent.parent
        xacro = (root / "src/mentorpi_description/urdf/mecanum.xacro").read_text()
        mesh_dir = root / "src/mentorpi_description/meshes/mecanum"

        def prop(name: str) -> float:
            m = re.search(
                rf'<xacro:property name="{name}"\s+value="([-0-9.]+)"', xacro)
            return float(m.group(1))

        def joint_origin(name: str):
            m = re.search(
                rf'<joint name="{name}".*?<origin xyz="([^"]+)" rpy="([^"]+)"',
                xacro, re.S)
            return ([float(v) for v in m.group(1).split()],
                    [float(v) for v in m.group(2).split()])

        x_off = prop("wheelbase") / 2
        y_off = prop("track_width") / 2
        wheel_z = prop("wheel_z")
        parts = [("base", mesh_dir / "base_link.STL", [0, 0, 0], [0, 0, 0])]
        for pfx, sx, sy in (("lf", 1, 1), ("rf", 1, -1),
                            ("lb", -1, 1), ("rb", -1, -1)):
            parts.append((f"wheel_{pfx}", mesh_dir / f"wheel_{pfx}_Link.STL",
                          [sx * x_off, sy * y_off, wheel_z], [0, 0, 0]))
        for part, joint, mesh in (("laser", "laser_joint", "lidar_Link.STL"),
                                  ("camera", "camera_joint", "cam_Link.STL")):
            xyz, rpy = joint_origin(joint)
            parts.append((part, mesh_dir / mesh, xyz, rpy))
        for _, mesh, _, _ in parts:
            if not mesh.is_file():
                raise FileNotFoundError(mesh)
        return parts

    def ensure(self, base_path: Optional[str]) -> None:
        if not self.parts or base_path is None or base_path == self.base_path:
            return
        if self.base_path is not None:
            rr.log(self.base_path + "/model", rr.Clear(recursive=True))
        self.base_path = base_path
        for name, mesh, xyz, rpy in self.parts:
            p = f"{base_path}/model/{name}"
            rr.log(p, rr.Transform3D(
                translation=xyz,
                rotation=rr.Quaternion(xyzw=quat_from_rpy(*rpy))), static=True)
            rr.log(p, rr.Asset3D(path=str(mesh)), static=True)


def quat_to_rot(q):
    x, y, z, w = q
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(R) -> np.ndarray:
    """Rotation matrix -> xyzw quaternion (Shepperd's method)."""
    t = float(np.trace(R))
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


# ---------- foxglove websocket transport (binary CDR, decode client-side) ----------

class FoxgloveClient:
    """Minimal Foxglove WebSocket protocol v1 client.

    foxglove_bridge forwards the DDS-serialized CDR payload untouched, so
    subscribing here costs the Pi almost nothing; we deserialize with
    `rosbags` locally and hand callbacks rosbridge-shaped dicts (so the
    same callbacks serve both transports).

    Binary MESSAGE_DATA frame layout:
        u8 opcode(=1) | u32le subscription_id | u64le receive_ns | CDR payload
    """

    def __init__(self, host: str, port: int) -> None:
        self.url = f"ws://{host}:{port}"
        # topic -> (callback, min_period_s, [last_emit_monotonic])
        self._subs: dict[str, tuple] = {}
        from rosbags.typesys import Stores, get_typestore
        self._ts = get_typestore(Stores.ROS2_JAZZY)
        self._on_ready = None

    def subscribe(self, topic: str, cb, min_period: float = 0.0) -> None:
        self._subs[topic] = (cb, min_period, [0.0])

    def on_ready(self, cb) -> None:
        self._on_ready = cb

    # -- rosbags message -> rosbridge-shaped dict (only the fields we use) --

    @staticmethod
    def _header(h) -> dict:
        return {"frame_id": h.frame_id,
                "stamp": {"sec": int(h.stamp.sec), "nanosec": int(h.stamp.nanosec)}}

    def _to_dict(self, msg, typename: str):
        if typename == "tf2_msgs/msg/TFMessage":
            return {"transforms": [{
                "header": self._header(t.header),
                "child_frame_id": t.child_frame_id,
                "transform": {
                    "translation": {"x": t.transform.translation.x,
                                    "y": t.transform.translation.y,
                                    "z": t.transform.translation.z},
                    "rotation": {"x": t.transform.rotation.x,
                                 "y": t.transform.rotation.y,
                                 "z": t.transform.rotation.z,
                                 "w": t.transform.rotation.w},
                },
            } for t in msg.transforms]}
        if typename == "sensor_msgs/msg/CompressedImage":
            return {"header": self._header(msg.header),
                    "format": msg.format,
                    "data": bytes(msg.data)}          # raw bytes, not base64
        if typename == "sensor_msgs/msg/CameraInfo":
            return {"header": self._header(msg.header),
                    "k": [float(v) for v in msg.k],
                    "width": int(msg.width), "height": int(msg.height)}
        if typename == "sensor_msgs/msg/Image":
            return {"header": self._header(msg.header),
                    "height": int(msg.height), "width": int(msg.width),
                    "encoding": msg.encoding,
                    "is_bigendian": bool(msg.is_bigendian),
                    "data": bytes(msg.data)}          # raw bytes, not base64
        return None

    def run_forever(self) -> None:
        from websockets.sync.client import connect

        while True:
            try:
                # foxglove_bridge >= 3.x (SDK-based, e.g. Jazzy 3.2.6) expects
                # "foxglove.sdk.v1"; older bridges expect the v1 string. Offer
                # both — the wire framing we rely on is identical.
                with connect(self.url,
                             subprotocols=["foxglove.sdk.v1",
                                           "foxglove.websocket.v1"],
                             max_size=None) as ws:
                    if self._on_ready:
                        self._on_ready()
                    # subscription id -> (topic, schemaName)
                    sub_of: dict[int, tuple[str, str]] = {}
                    subscribed_channels: set[int] = set()
                    next_sub_id = [0]

                    for raw in ws:
                        if isinstance(raw, str):
                            m = json.loads(raw)
                            if m.get("op") != "advertise":
                                continue
                            reqs = []
                            for ch in m.get("channels", []):
                                topic = ch.get("topic")
                                if (topic in self._subs
                                        and ch.get("encoding") == "cdr"
                                        and ch["id"] not in subscribed_channels):
                                    sid = next_sub_id[0]
                                    next_sub_id[0] += 1
                                    sub_of[sid] = (topic, ch["schemaName"])
                                    subscribed_channels.add(ch["id"])
                                    reqs.append({"id": sid, "channelId": ch["id"]})
                            if reqs:
                                ws.send(json.dumps(
                                    {"op": "subscribe", "subscriptions": reqs}))
                            continue

                        if not raw or raw[0] != 0x01 or len(raw) < 13:
                            continue
                        sid = struct.unpack_from("<I", raw, 1)[0]
                        entry = sub_of.get(sid)
                        if entry is None:
                            continue
                        topic, schema = entry
                        cb, min_period, last = self._subs[topic]
                        now = time.monotonic()
                        if min_period > 0.0 and now - last[0] < min_period:
                            continue  # client-side throttle: drop before decode
                        last[0] = now
                        try:
                            msg = self._ts.deserialize_cdr(bytes(raw[13:]), schema)
                        except Exception:
                            continue
                        d = self._to_dict(msg, schema)
                        if d is not None:
                            cb(d)
            except KeyboardInterrupt:
                raise
            except Exception as exc:  # bridge restart / WiFi blip: retry
                print(f"foxglove connection lost ({exc}); retrying in 3s")
                time.sleep(3)


# ---------- main ----------

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", required=True, help="robot hostname / IP")
    ap.add_argument("--transport", choices=["foxglove", "rosbridge"],
                    default="foxglove",
                    help="foxglove = binary CDR via foxglove_bridge :8765 "
                         "(Pi-cheap, default); rosbridge = legacy JSON :9090")
    ap.add_argument("--port", type=int, default=None,
                    help="bridge port (default: 8765 foxglove / 9090 rosbridge)")
    ap.add_argument("--splat", type=Path, default=None,
                    help="offline Gaussian-splat .ply (map frame)")
    ap.add_argument("--cloud", type=Path, default=None,
                    help="rtabmap exported cloud .ply (map frame)")
    ap.add_argument("--point-radius", type=float, default=0.008)
    ap.add_argument("--depth-cloud", action="store_true",
                    help="live true-color RGB-D point cloud (backprojects "
                         "/viewer/depth_raw 2Hz + latest RGB, ~2ms/frame)")
    ap.add_argument("--depth-stride", type=int, default=2,
                    help="depth pixel subsampling (2 -> 320x240 = 76k pts)")
    ap.add_argument("--depth-max", type=float, default=4.0,
                    help="discard depth beyond this range (m)")
    ap.add_argument("--image-hz", type=float, default=4.0,
                    help="max live camera image decode rate (client-side)")
    ap.add_argument("--image-topic", default="/viewer/color_compressed",
                    help="compressed image topic; default is the robot-side "
                         "2Hz throttled stream (WiFi-friendly). Use "
                         "/camera/color/image_raw/compressed for full rate")
    ap.add_argument("--serve", action="store_true",
                    help="serve the Rerun web viewer (phone/tablet browsers)")
    args = ap.parse_args()

    rr.init("mentorpi_live")
    if args.serve:
        # server_memory_limit 默认高达内存 75%: 服务端会攒下完整历史,
        # 每个新连上的 viewer 都要从头回灌几分钟的旧数据才追上实时,
        # 看起来就是"巨额延迟 + 在播过去"。64MB 只保留最近几十秒。
        try:
            try:
                server_uri = rr.serve_grpc(server_memory_limit="64MB")
            except TypeError:
                server_uri = rr.serve_grpc()
            rr.serve_web_viewer(connect_to=server_uri)
        except AttributeError:  # rerun < 0.24: no serve_grpc/serve_web_viewer
            try:
                rr.serve_web(open_browser=False, server_memory_limit="64MB")
            except TypeError:
                rr.serve_web()
        print("\nopen the printed URL on your phone/PC (same LAN)\n")
    else:
        rr.spawn()

    # ROS convention at the root: X forward, Y left, Z up.
    rr.log("/", rr.ViewCoordinates.FLU, static=True)

    for ply_path, entity, radius in ((args.splat, f"{MAP}/splat", args.point_radius),
                                     (args.cloud, f"{MAP}/slam_cloud", args.point_radius)):
        if ply_path is None:
            continue
        print(f"loading {ply_path} ...")
        xyz, rgb = load_ply_points(ply_path.expanduser())
        print(f"  {len(xyz):,} points")
        rr.log(entity, rr.Points3D(xyz, colors=rgb, radii=radius), static=True)

    tree = TfTree()
    robot = RobotModel()
    trajectory: list[list[float]] = []
    last_traj_log = [0.0]

    def on_tf(msg, static: bool):
        for tf in msg["transforms"]:
            t = tf["transform"]["translation"]
            q = tf["transform"]["rotation"]
            stamp = tf["header"]["stamp"]
            child = tf["child_frame_id"].lstrip("/")
            parent = tf["header"]["frame_id"].lstrip("/")
            tree.update(
                child, parent,
                np.array([t["x"], t["y"], t["z"]]),
                np.array([q["x"], q["y"], q["z"], q["w"]]),
                static=static,
                stamp=stamp["sec"] + stamp["nanosec"] * 1e-9,
            )
            # Grow trajectory from map->base_link whenever odom pose moves.
            if child == "base_link" and not static:
                robot.ensure(tree.path("base_link"))
                M = tree.map_from("base_link")
                if M is not None:
                    p = M[:3, 3]
                    if (not trajectory
                            or np.linalg.norm(p - np.array(trajectory[-1])) > 0.02):
                        trajectory.append(p.tolist())
                    now = time.monotonic()
                    if now - last_traj_log[0] > 0.5 and len(trajectory) >= 2:
                        last_traj_log[0] = now
                        rr.log(f"{MAP}/trajectory",
                               rr.LineStrips3D([np.asarray(trajectory, dtype=np.float32)],
                                               colors=[40, 200, 255], radii=0.008))

    cam_k: dict[str, np.ndarray] = {}       # frame -> 3x3 K
    latest_jpeg: list[Optional[bytes]] = [None]   # newest RGB, decoded lazily

    def on_camera_info(msg):
        frame = msg["header"]["frame_id"].lstrip("/")
        path = tree.path(frame)
        if path is None:
            return
        K = np.array(msg["k"], dtype=np.float64).reshape(3, 3)
        cam_k[frame] = K
        rr.log(path, rr.Pinhole(
            image_from_camera=K,
            resolution=[int(msg["width"]), int(msg["height"])],
            camera_xyz=rr.ViewCoordinates.RDF,   # optical frame
            image_plane_distance=0.4,
        ), static=True)

    def on_compressed_image(msg):
        frame = msg["header"]["frame_id"].lstrip("/")
        path = tree.path(frame)
        if path is None:
            return
        stamp = msg["header"]["stamp"]
        set_ros_time(stamp["sec"] + stamp["nanosec"] * 1e-9)
        data = msg["data"]
        # foxglove path hands us raw bytes; rosbridge JSON hands base64 str.
        jpeg = data if isinstance(data, (bytes, bytearray)) else base64.b64decode(data)
        latest_jpeg[0] = jpeg
        fmt = "image/png" if "png" in msg.get("format", "") else "image/jpeg"
        rr.log(path + "/image", rr.EncodedImage(contents=jpeg, media_type=fmt))

    depth_warned: list[bool] = [False]
    depth_started: list[bool] = [False]

    def on_depth(msg):
        # 真彩 RGB-D 反投影: /viewer/depth_raw (2Hz lazy 节流) + 最近一帧
        # RGB。深度已 HW 对齐到 color (同 K 同 frame),像素 1:1 取色。点在
        # 客户端转到 map 系后记录在固定实体 —— 避开 Pinhole 子实体的 2D
        # 语义,也不依赖 viewer 端 transform 时序。
        frame = msg["header"]["frame_id"].lstrip("/")
        K = cam_k.get(frame)
        if K is None:
            return                       # camera_info not seen yet
        if msg.get("encoding") != "16UC1":
            if not depth_warned[0]:
                depth_warned[0] = True
                print(f"depth cloud: unsupported encoding {msg.get('encoding')}")
            return
        data = msg["data"]
        raw = data if isinstance(data, (bytes, bytearray)) else base64.b64decode(data)
        h, w = int(msg["height"]), int(msg["width"])
        dt = ">u2" if msg.get("is_bigendian") else "<u2"
        depth_mm = np.frombuffer(raw, dtype=dt).reshape(h, w)
        rgb = decode_jpeg_rgb(latest_jpeg[0]) if latest_jpeg[0] else None
        pts, colors = backproject_depth(depth_mm, K, rgb,
                                        args.depth_stride, args.depth_max)
        if not len(pts):
            return
        M = tree.map_from(frame)
        if M is None:
            return
        pts = pts @ M[:3, :3].T + M[:3, 3]
        stamp = msg["header"]["stamp"]
        set_ros_time(stamp["sec"] + stamp["nanosec"] * 1e-9)
        rr.log(f"{MAP}/depth_cloud",
               rr.Points3D(pts.astype(np.float32), colors=colors, radii=0.006))
        if not depth_started[0]:
            depth_started[0] = True
            print(f"depth cloud active: {len(pts):,} pts/frame "
                  f"({'true color' if colors is not None else 'no RGB yet'})")

    image_period = 1.0 / max(args.image_hz, 0.1)

    if args.transport == "foxglove":
        port = args.port or 8765
        fox = FoxgloveClient(args.robot, port)
        fox.subscribe("/tf_static", lambda m: on_tf(m, True))
        # foxglove_bridge 是二进制直通, Pi 侧无逐条序列化开销 —— TF 可以
        # 全速吃, 位姿丝滑; 图像按 image_hz 客户端丢帧(解码前就丢)。
        fox.subscribe("/tf", lambda m: on_tf(m, False))
        fox.subscribe("/camera/color/camera_info", on_camera_info, min_period=2.0)
        fox.subscribe(args.image_topic, on_compressed_image,
                      min_period=image_period)
        if args.depth_cloud:
            fox.subscribe("/viewer/depth_raw", on_depth, min_period=0.45)
        fox.on_ready(lambda: print(f"connected to ws://{args.robot}:{port} (foxglove)"))
        print("connecting ... (Ctrl-C to quit)")
        try:
            fox.run_forever()
        except KeyboardInterrupt:
            pass
        return 0

    # legacy rosbridge transport
    import roslibpy
    port = args.port or 9090
    ros = roslibpy.Ros(host=args.robot, port=port)
    roslibpy.Topic(ros, "/tf_static", "tf2_msgs/msg/TFMessage",
                   queue_length=1).subscribe(lambda m: on_tf(m, True))
    # /tf 全量是 50Hz+ (EKF), rosbridge (纯 Python) 逐条 JSON 序列化扛
    # 不住, 在 rosbridge 侧限到 10Hz —— 位姿显示足够流畅, 开销降 5 倍。
    roslibpy.Topic(ros, "/tf", "tf2_msgs/msg/TFMessage",
                   throttle_rate=100,
                   queue_length=1).subscribe(lambda m: on_tf(m, False))
    roslibpy.Topic(ros, "/camera/color/camera_info", "sensor_msgs/msg/CameraInfo",
                   throttle_rate=2000, queue_length=1).subscribe(on_camera_info)
    roslibpy.Topic(ros, args.image_topic,
                   "sensor_msgs/msg/CompressedImage",
                   throttle_rate=int(1000 * image_period),
                   queue_length=1).subscribe(on_compressed_image)
    if args.depth_cloud:
        roslibpy.Topic(ros, "/viewer/depth_raw", "sensor_msgs/msg/Image",
                       throttle_rate=450,
                       queue_length=1).subscribe(on_depth)

    ros.on_ready(lambda: print(f"connected to ws://{args.robot}:{port} (rosbridge)"))
    print("connecting ... (Ctrl-C to quit)")
    try:
        ros.run_forever()
    except KeyboardInterrupt:
        ros.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
