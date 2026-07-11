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
import json
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rerun as rr

MAP = "map"  # rerun root entity == ROS map frame


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


# ---------- TF tree ----------

class TfTree:
    """Keeps latest transform per edge and mirrors the chain into Rerun.

    Rerun entity paths are built from the TF parent chain (map/odom/base_link/
    ...). TF messages can arrive before their parent links are known (the
    orbbec driver publishes deep optical frames before camera_link), which
    would pin entities to wrong paths — so whenever the topology changes we
    re-log every known edge at its recomputed path. The tree is ~10 frames,
    this is cheap.
    """

    def __init__(self) -> None:
        self.parent_of: dict[str, str] = {}
        # child -> (translation, quaternion, is_static)
        self.xform: dict[str, tuple[np.ndarray, np.ndarray, bool]] = {}
        self.lock = threading.Lock()

    def _path_locked(self, frame: str) -> Optional[str]:
        chain, seen = [], set()
        cur: Optional[str] = frame
        while cur is not None:
            if cur in seen:
                return None  # cycle
            seen.add(cur)
            chain.append(cur)
            cur = self.parent_of.get(cur)
        if chain[-1] != MAP:
            # Not yet connected to map (e.g. no rtabmap map->odom): hang the
            # orphan chain under map anyway so the robot is still visible.
            chain.append(MAP)
        chain.reverse()
        return "/".join(chain)

    def path(self, frame: str) -> Optional[str]:
        with self.lock:
            return self._path_locked(frame)

    @staticmethod
    def _log_edge(path: str, t: np.ndarray, q: np.ndarray, static: bool) -> None:
        rr.log(path, rr.Transform3D(translation=t, rotation=rr.Quaternion(xyzw=q)),
               static=static)

    def update(self, child: str, parent: str, t: np.ndarray, q: np.ndarray,
               *, static: bool, stamp: float) -> None:
        to_log: list[tuple[str, np.ndarray, np.ndarray, bool]] = []
        with self.lock:
            topology_changed = self.parent_of.get(child) != parent
            self.parent_of[child] = parent
            self.xform[child] = (t, q, static)
            if topology_changed:
                # Re-log everything at (possibly) new paths.
                for c, (ct, cq, cs) in self.xform.items():
                    p = self._path_locked(c)
                    if p is not None:
                        to_log.append((p, ct, cq, cs))
            else:
                p = self._path_locked(child)
                if p is not None:
                    to_log.append((p, t, q, static))
        if not static:
            rr.set_time("ros_time", timestamp=stamp)
        for p, ct, cq, cs in to_log:
            self._log_edge(p, ct, cq, cs)

    def map_from(self, frame: str) -> Optional[np.ndarray]:
        """Compose map<-frame as a 4x4 by walking stored edges. An orphan
        chain root is treated as sitting at the map origin (consistent with
        the entity paths above)."""
        with self.lock:
            M = np.eye(4)
            cur = frame
            while cur != MAP:
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


def quat_to_rot(q):
    x, y, z, w = q
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


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
        return None

    def run_forever(self) -> None:
        from websockets.sync.client import connect

        while True:
            try:
                with connect(self.url, subprotocols=["foxglove.websocket.v1"],
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
    ap.add_argument("--image-hz", type=float, default=4.0,
                    help="live camera image rate (throttled robot-side)")
    ap.add_argument("--serve", action="store_true",
                    help="serve the Rerun web viewer (phone/tablet browsers)")
    args = ap.parse_args()

    rr.init("mentorpi_live")
    if args.serve:
        try:
            server_uri = rr.serve_grpc()
            rr.serve_web_viewer(connect_to=server_uri)
        except AttributeError:
            rr.serve_web()  # rerun < 0.24 API
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

    def on_camera_info(msg):
        frame = msg["header"]["frame_id"].lstrip("/")
        path = tree.path(frame)
        if path is None:
            return
        K = np.array(msg["k"], dtype=np.float64).reshape(3, 3)
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
        rr.set_time("ros_time", timestamp=stamp["sec"] + stamp["nanosec"] * 1e-9)
        data = msg["data"]
        # foxglove path hands us raw bytes; rosbridge JSON hands base64 str.
        jpeg = data if isinstance(data, (bytes, bytearray)) else base64.b64decode(data)
        fmt = "image/png" if "png" in msg.get("format", "") else "image/jpeg"
        rr.log(path + "/image", rr.EncodedImage(contents=jpeg, media_type=fmt))

    image_period = 1.0 / max(args.image_hz, 0.1)

    if args.transport == "foxglove":
        port = args.port or 8765
        fox = FoxgloveClient(args.robot, port)
        fox.subscribe("/tf_static", lambda m: on_tf(m, True))
        # foxglove_bridge 是二进制直通, Pi 侧无逐条序列化开销 —— TF 可以
        # 全速吃, 位姿丝滑; 图像按 image_hz 客户端丢帧(解码前就丢)。
        fox.subscribe("/tf", lambda m: on_tf(m, False))
        fox.subscribe("/camera/color/camera_info", on_camera_info, min_period=2.0)
        fox.subscribe("/camera/color/image_raw/compressed", on_compressed_image,
                      min_period=image_period)
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
    roslibpy.Topic(ros, "/camera/color/image_raw/compressed",
                   "sensor_msgs/msg/CompressedImage",
                   throttle_rate=int(1000 * image_period),
                   queue_length=1).subscribe(on_compressed_image)

    ros.on_ready(lambda: print(f"connected to ws://{args.robot}:{port} (rosbridge)"))
    print("connecting ... (Ctrl-C to quit)")
    try:
        ros.run_forever()
    except KeyboardInterrupt:
        ros.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
