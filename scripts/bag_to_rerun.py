#!/usr/bin/env python3
"""
Replay a MentorPi 3D-SLAM rosbag into Rerun for offline debugging.

Streams a recorded ROS 2 bag (MCAP or SQLite3) into the Rerun viewer:
  - TF tree (static + dynamic) becomes a Rerun Transform3D hierarchy
  - /camera/color/camera_info     -> rr.Pinhole on the optical frame
                                     (camera position + FOV frustum)
  - /camera/color/image_raw       -> rr.Image attached to the color frustum
  - /camera/depth/image_raw       -> rr.DepthImage attached to its frustum
  - /scan                         -> rr.Points3D under laser_frame
  - /odometry/filtered            -> running trajectory polyline + scalars
  - /odom, /imu/data*, /cmd_vel   -> scalar timeseries

The bag is the one produced by `record_3d_bag.sh`; missing topics are
silently skipped, so partial bags also work.

Dependencies (remote / desktop machine, no ROS install needed):
    pip install rerun-sdk rosbags numpy

Usage:
    python3 bag_to_rerun.py ~/rosbags/3d_slam/<run_name>
    python3 bag_to_rerun.py ~/rosbags/3d_slam/<run_name> --save out.rrd
"""

import argparse
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import rerun as rr
from rosbags.highlevel import AnyReader

WORLD = "world"  # Rerun root for the spatial tree


# ---------- TF tree ------------------------------------------------------

class FrameTree:
    """Map TF frame names to Rerun entity paths by walking parent links."""

    def __init__(self) -> None:
        self.parent_of: dict[str, str] = {}

    def set_parent(self, child: str, parent: str) -> None:
        self.parent_of[child] = parent

    def path(self, frame: str) -> Optional[str]:
        chain: list[str] = []
        seen: set[str] = set()
        cur: Optional[str] = frame
        while cur is not None:
            if cur in seen:
                return None  # cycle
            seen.add(cur)
            chain.append(cur)
            cur = self.parent_of.get(cur)
        chain.reverse()
        return WORLD + "/" + "/".join(chain)


# ---------- helpers ------------------------------------------------------

def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def set_ros_time(seconds: float) -> None:
    # Rerun >= 0.21 dropped set_time_seconds in favour of set_time(timestamp=...).
    rr.set_time("ros_time", timestamp=seconds)


def transform_components(tf):
    t = tf.transform.translation
    q = tf.transform.rotation
    return (
        np.array([t.x, t.y, t.z], dtype=np.float64),
        np.array([q.x, q.y, q.z, q.w], dtype=np.float64),
    )


def clean_frame(frame_id: str) -> str:
    return frame_id.lstrip("/")


# ---------- handlers -----------------------------------------------------

def handle_tf(msg, tree: FrameTree, *, static: bool) -> None:
    for tf in msg.transforms:
        child = clean_frame(tf.child_frame_id)
        parent = clean_frame(tf.header.frame_id)
        tree.set_parent(child, parent)
        path = tree.path(child)
        if path is None:
            continue
        translation, quat = transform_components(tf)
        xform = rr.Transform3D(
            translation=translation,
            rotation=rr.Quaternion(xyzw=quat),
        )
        if static:
            rr.log(path, xform, static=True)
        else:
            set_ros_time(stamp_seconds(tf.header.stamp))
            rr.log(path, xform)


def handle_camera_info(msg, tree: FrameTree) -> None:
    """Log Pinhole on the optical frame so the camera frustum + FOV show up."""
    frame = clean_frame(msg.header.frame_id)
    base = tree.path(frame)
    if base is None:
        return
    K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
    rr.log(
        base,
        rr.Pinhole(
            image_from_camera=K,
            resolution=[int(msg.width), int(msg.height)],
            camera_xyz=rr.ViewCoordinates.RDF,  # optical frame: x-right y-down z-forward
        ),
        static=True,
    )


def handle_image(msg, tree: FrameTree, kind: str) -> None:
    """kind in {'rgb', 'depth'}. Path: <optical_frame>/{image|depth}."""
    frame = clean_frame(msg.header.frame_id)
    base = tree.path(frame)
    if base is None:
        return
    set_ros_time(stamp_seconds(msg.header.stamp))
    h, w = int(msg.height), int(msg.width)
    enc = msg.encoding.lower() if isinstance(msg.encoding, str) else msg.encoding

    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    if kind == "rgb":
        if enc == "rgb8":
            img = buf.reshape(h, w, 3)
        elif enc == "bgr8":
            img = buf.reshape(h, w, 3)[..., ::-1]
        elif enc == "mono8":
            img = buf.reshape(h, w)
        elif enc in ("rgba8", "bgra8"):
            img = buf.reshape(h, w, 4)[..., :3]
            if enc.startswith("bgr"):
                img = img[..., ::-1]
        else:
            print(f"[warn] unsupported color encoding: {enc}", file=sys.stderr)
            return
        rr.log(base + "/image", rr.Image(img))
    else:  # depth
        if enc == "16uc1":
            img = buf.view(np.uint16).reshape(h, w)
            rr.log(base + "/depth", rr.DepthImage(img, meter=1000.0))
        elif enc == "32fc1":
            img = buf.view(np.float32).reshape(h, w)
            rr.log(base + "/depth", rr.DepthImage(img, meter=1.0))
        else:
            print(f"[warn] unsupported depth encoding: {enc}", file=sys.stderr)


def handle_laser_scan(msg, tree: FrameTree) -> None:
    frame = clean_frame(msg.header.frame_id)
    base = tree.path(frame)
    if base is None:
        return
    ranges = np.asarray(msg.ranges, dtype=np.float32)
    angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment
    valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
    angles = angles[valid]
    ranges = ranges[valid]
    points = np.stack(
        [ranges * np.cos(angles), ranges * np.sin(angles), np.zeros_like(ranges)],
        axis=1,
    )
    set_ros_time(stamp_seconds(msg.header.stamp))
    rr.log(
        base + "/scan",
        rr.Points3D(points, colors=[230, 200, 40], radii=0.02),
    )


def handle_imu(msg, ns: str) -> None:
    set_ros_time(stamp_seconds(msg.header.stamp))
    a = msg.linear_acceleration
    g = msg.angular_velocity
    rr.log(f"/diagnostics/{ns}/accel/x", rr.Scalars(a.x))
    rr.log(f"/diagnostics/{ns}/accel/y", rr.Scalars(a.y))
    rr.log(f"/diagnostics/{ns}/accel/z", rr.Scalars(a.z))
    rr.log(f"/diagnostics/{ns}/gyro/x", rr.Scalars(g.x))
    rr.log(f"/diagnostics/{ns}/gyro/y", rr.Scalars(g.y))
    rr.log(f"/diagnostics/{ns}/gyro/z", rr.Scalars(g.z))
    # If orientation is populated (Madgwick output), expose the quaternion magnitude
    # so we can spot dropouts; the actual orientation is already in the TF tree.
    if msg.orientation_covariance[0] >= 0.0:
        q = msg.orientation
        rr.log(
            f"/diagnostics/{ns}/orientation_norm",
            rr.Scalars(float(np.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w))),
        )


def handle_odometry(msg, ns: str, traj_buf: Optional[list]) -> None:
    set_ros_time(stamp_seconds(msg.header.stamp))
    v = msg.twist.twist
    rr.log(f"/diagnostics/{ns}/v_lin/x", rr.Scalars(v.linear.x))
    rr.log(f"/diagnostics/{ns}/v_lin/y", rr.Scalars(v.linear.y))
    rr.log(f"/diagnostics/{ns}/v_ang/z", rr.Scalars(v.angular.z))
    if traj_buf is not None:
        p = msg.pose.pose.position
        traj_buf.append([p.x, p.y, p.z])
        # Re-log the polyline so it grows over time.
        rr.log(
            f"/{WORLD}/odom_path",
            rr.LineStrips3D(
                [np.asarray(traj_buf, dtype=np.float32)],
                colors=[40, 200, 255],
                radii=0.01,
            ),
        )


def handle_cmd_vel(msg, t_seconds: float) -> None:
    # Twist has no header — use the bag-receive timestamp.
    set_ros_time(t_seconds)
    rr.log("/diagnostics/cmd_vel/v_lin/x", rr.Scalars(msg.linear.x))
    rr.log("/diagnostics/cmd_vel/v_lin/y", rr.Scalars(msg.linear.y))
    rr.log("/diagnostics/cmd_vel/v_ang/z", rr.Scalars(msg.angular.z))


# ---------- main ---------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="rosbag2 directory (MCAP or SQLite3)")
    ap.add_argument("--app-id", default="mentorpi_3d_slam")
    ap.add_argument(
        "--save",
        default=None,
        help="write to this .rrd file instead of spawning the viewer",
    )
    args = ap.parse_args()

    bag_path = Path(args.bag).expanduser()
    if not bag_path.exists():
        print(f"bag not found: {bag_path}", file=sys.stderr)
        return 2

    rr.init(args.app_id, spawn=(args.save is None))
    if args.save:
        rr.save(args.save)

    # ROS REP-103 ground convention: X forward, Y left, Z up.
    rr.log("/", rr.ViewCoordinates.FLU, static=True)

    tree = FrameTree()
    odom_traj: list[list[float]] = []
    seen: dict[str, int] = {}

    # ---- Pass 1: build the full TF parent map ----
    # Static TF messages are emitted in arbitrary order (orbbec driver publishes
    # `camera_link → camera_depth_frame` AFTER the deeper optical frames). If we
    # logged Transform3D on the fly while the chain is partial, the static
    # Transform3Ds land at orphan paths and the camera ends up disconnected from
    # base_link, so back-projected depth points go to world +Z.
    print("scanning TF tree...")
    with AnyReader([bag_path]) as reader:
        tf_conns = [c for c in reader.connections
                    if c.topic in ("/tf_static", "/tf")]
        for conn, _t, raw in reader.messages(connections=tf_conns):
            msg = reader.deserialize(raw, conn.msgtype)
            for tf in msg.transforms:
                tree.set_parent(clean_frame(tf.child_frame_id),
                                clean_frame(tf.header.frame_id))
    print(f"  resolved {len(tree.parent_of)} frames")

    # ---- Pass 2: log everything ----
    with AnyReader([bag_path]) as reader:
        topic_types = {c.topic: c.msgtype for c in reader.connections}
        print("topics in bag:")
        for t in sorted(topic_types):
            print(f"  {t:42s} {topic_types[t]}")
        print()

        for conn, t_ns, raw in reader.messages():
            topic = conn.topic
            seen[topic] = seen.get(topic, 0) + 1
            try:
                msg = reader.deserialize(raw, conn.msgtype)
            except Exception as e:
                print(f"[warn] deserialize {topic}: {e}", file=sys.stderr)
                continue

            if topic == "/tf_static":
                handle_tf(msg, tree, static=True)
            elif topic == "/tf":
                handle_tf(msg, tree, static=False)
            elif topic in (
                "/camera/color/camera_info",
                "/camera/depth/camera_info",
            ):
                handle_camera_info(msg, tree)
            elif topic == "/camera/color/image_raw":
                handle_image(msg, tree, kind="rgb")
            elif topic == "/camera/depth/image_raw":
                handle_image(msg, tree, kind="depth")
            elif topic == "/scan":
                handle_laser_scan(msg, tree)
            elif topic == "/imu/data":
                handle_imu(msg, ns="imu_filtered")
            elif topic == "/imu/data_raw":
                handle_imu(msg, ns="imu_raw")
            elif topic == "/odometry/filtered":
                handle_odometry(msg, ns="odom_filtered", traj_buf=odom_traj)
            elif topic == "/odom":
                handle_odometry(msg, ns="odom_wheel", traj_buf=None)
            elif topic == "/cmd_vel":
                handle_cmd_vel(msg, t_ns * 1e-9)

    print("\nmessage counts:")
    for topic in sorted(seen):
        print(f"  {topic:42s} {seen[topic]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
