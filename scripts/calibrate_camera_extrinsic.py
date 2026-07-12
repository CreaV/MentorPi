#!/usr/bin/env python3
"""AprilTag hand-eye calibration for the base_link -> camera extrinsic.

Setup: a tag36h11 #0 displayed on a tablet standing against a wall (tilt is
fine — the tag pose is solved, not assumed). Robot starts anywhere with the
tag roughly in view range (it scans in place to find it).

Procedure (all automatic over rosbridge):
  1. scan: rotate in 30° steps until the tag is detected
  2. collect: visit a grid of stations (lateral strafe x depth), re-aiming
     the tag to image center after each move; at each station capture
     (EKF odom pose, camera frame, PnP tag pose) standing still
  3. solve: least-squares over T_base_cam (z FIXED to the ruler-measured
     height — planar robot motion cannot observe it) and T_odom_tag
  4. print the base_link -> camera_link translation+RPY for base.launch.py

Run (dev machine, aire-venv has pupil_apriltags/scipy/websockets):
  python scripts/calibrate_camera_extrinsic.py ws://192.168.8.117:9090 \
      --tag-size 0.1175 --out /tmp/calib_run
  python scripts/calibrate_camera_extrinsic.py --solve-only /tmp/calib_run

The robot must be in a mode with base running (idle is perfect); motion uses
the bounded primitives, so the obstacle guard stays in effect throughout.
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import io
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

AIRE_PATH_DEFAULT = "/media/luo/Game/data/code/AIRE"


# ---------- SE(3) helpers ----------

def rt_to_mat(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


def quat_xyzw_to_mat(q, t) -> np.ndarray:
    from scipy.spatial.transform import Rotation
    return rt_to_mat(Rotation.from_quat(q).as_matrix(), t)


def mat_to_rpy(T) -> tuple:
    from scipy.spatial.transform import Rotation
    r = Rotation.from_matrix(T[:3, :3])
    roll, pitch, yaw = r.as_euler('xyz')
    return roll, pitch, yaw


# optical(z forward, x right, y down) -> link(x forward, y left, z up)
M_LINK_FROM_OPT = np.array([[0.0, 0.0, 1.0],
                            [-1.0, 0.0, 0.0],
                            [0.0, -1.0, 0.0]])


# ---------- robot I/O ----------

class Robot:
    def __init__(self, url: str, aire_path: str):
        sys.path.insert(0, aire_path)
        from air_engine.cloud.robot.rosbridge_client import RosbridgeClient
        from air_engine.cloud.robot.tools import RobotToolRouter
        self.client = RosbridgeClient(url)
        self.router = RobotToolRouter(self.client)

    async def camera_info(self):
        msg = await self.client.subscribe_once(
            "/camera/color/camera_info", "sensor_msgs/msg/CameraInfo", timeout=10)
        k = msg["k"]
        return float(k[0]), float(k[4]), float(k[2]), float(k[5])  # fx fy cx cy

    async def frame(self) -> bytes:
        msg = await self.client.subscribe_once(
            "/viewer/color_compressed", "sensor_msgs/msg/CompressedImage", timeout=10)
        data = msg["data"]
        return base64.b64decode(data) if isinstance(data, str) else bytes(data)

    async def odom(self):
        msg = await self.client.subscribe_once(
            "/odometry/filtered", "nav_msgs/msg/Odometry", timeout=5)
        p = msg["pose"]["pose"]["position"]
        q = msg["pose"]["pose"]["orientation"]
        return quat_xyzw_to_mat([q["x"], q["y"], q["z"], q["w"]],
                                [p["x"], p["y"], p["z"]])

    async def rotate(self, deg: float) -> bool:
        r = await self.router.handle_tool_call("robot.rotate", {"angle_deg": deg})
        return bool(r.get("success"))

    async def move(self, direction: str, dist: float) -> bool:
        r = await self.router.handle_tool_call(
            "robot.move", {"direction": direction, "distance": dist, "speed": 0.15})
        return bool(r.get("success"))


class TagDetector:
    def __init__(self, fx, fy, cx, cy, tag_size: float):
        from pupil_apriltags import Detector
        self.det = Detector(families="tag36h11")
        self.params = (fx, fy, cx, cy)
        self.tag_size = tag_size

    def detect(self, jpeg: bytes):
        """Returns (T_cam_tag 4x4, center_u) or None."""
        from PIL import Image
        img = np.asarray(Image.open(io.BytesIO(jpeg)).convert("L"))
        dets = self.det.detect(img, estimate_tag_pose=True,
                               camera_params=self.params, tag_size=self.tag_size)
        best = None
        for d in dets:
            if d.tag_id == 0 and d.hamming == 0 and (
                    best is None or d.decision_margin > best.decision_margin):
                best = d
        if best is None:
            return None
        return rt_to_mat(best.pose_R, best.pose_t), float(best.center[0])


# ---------- collection ----------

async def aim_tag(robot, det, fx, cx, tries=3):
    """Rotate so the tag sits near image center; return last detection."""
    for _ in range(tries):
        await asyncio.sleep(1.0)
        r = det.detect(await robot.frame())
        if r is None:
            return None
        _, u = r
        ang = math.degrees(math.atan2(cx - u, fx))
        if abs(ang) < 4.0:
            return r
        await robot.rotate(ang)
    await asyncio.sleep(1.0)
    return det.detect(await robot.frame())


async def capture(robot, det, out: Path, records: list, note: str):
    await asyncio.sleep(1.2)  # settle + let the 2Hz viewer frame refresh
    T_odom_base = await robot.odom()
    jpeg = await robot.frame()
    r = det.detect(jpeg)
    if r is None:
        print(f"  [{note}] tag not detected — skipped")
        return
    T_cam_tag, u = r
    idx = len(records)
    (out / f"frame_{idx:02d}.jpg").write_bytes(jpeg)
    records.append({"note": note,
                    "T_odom_base": T_odom_base.tolist(),
                    "T_cam_tag": T_cam_tag.tolist()})
    print(f"  [{note}] captured #{idx} (tag u={u:.0f}px)")


async def collect(url: str, aire_path: str, tag_size: float, out: Path):
    robot = Robot(url, aire_path)
    fx, fy, cx, cy = await robot.camera_info()
    print(f"camera: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")
    det = TagDetector(fx, fy, cx, cy, tag_size)

    # -- scan for the tag --
    found = det.detect(await robot.frame())
    turns = 0
    while found is None and turns < 12:
        print("scanning: rotate 30°")
        await robot.rotate(30)
        await asyncio.sleep(1.0)
        found = det.detect(await robot.frame())
        turns += 1
    if found is None:
        print("!! tag not found after full circle — check tablet placement/glare")
        return False
    print("tag found; aiming")
    if await aim_tag(robot, det, fx, cx) is None:
        print("!! lost tag while aiming")
        return False

    records: list = []
    out.mkdir(parents=True, exist_ok=True)

    # -- stations: (label, how to get there from previous) --
    # Lateral sweep at the current depth, then a step back, sweep again.
    # At each station: aim, capture at -10°/0°/+10° headings.
    async def station(label):
        if await aim_tag(robot, det, fx, cx) is None:
            print(f"  [{label}] tag lost — station skipped")
            return
        await capture(robot, det, out, records, f"{label} aim")
        for d in (-10, 20):  # -10° then back through center to +10°
            await robot.rotate(d)
            await capture(robot, det, out, records, f"{label} {'-10' if d == -10 else '+10'}°")
        await robot.rotate(-10)  # back to aim

    await station("S0 center")
    if await robot.move("left", 0.3):
        await station("S1 left")
    if await robot.move("right", 0.6):
        await station("S2 right")
    if await robot.move("left", 0.3):
        pass  # back to center line
    if await robot.move("backward", 0.35):
        await station("S3 back-center")
        if await robot.move("left", 0.35):
            await station("S4 back-left")
        if await robot.move("right", 0.7):
            await station("S5 back-right")

    (out / "records.json").write_text(json.dumps(records, indent=1))
    print(f"collected {len(records)} poses -> {out}/records.json")
    await robot.client.close()
    return len(records) >= 8


# ---------- solve ----------

def solve(out: Path, cam_z: float):
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation

    records = json.loads((out / "records.json").read_text())
    T_ob = [np.array(r["T_odom_base"]) for r in records]
    T_ct = [np.array(r["T_cam_tag"]) for r in records]
    n = len(records)
    print(f"solving with {n} poses (camera z fixed at {cam_z} m)")

    # params: base->cam_optical [x y rvec(3)] + odom->tag [x y z rvec(3)]
    R0 = M_LINK_FROM_OPT  # camera level, no mount rotation
    x0 = np.zeros(11)
    x0[0:2] = [0.143, 0.0]
    x0[2:5] = Rotation.from_matrix(R0).as_rotvec()
    T_bc0 = rt_to_mat(R0, [0.143, 0.0, cam_z])
    T_ot0 = T_ob[0] @ T_bc0 @ T_ct[0]
    x0[5:8] = T_ot0[:3, 3]
    x0[8:11] = Rotation.from_matrix(T_ot0[:3, :3]).as_rotvec()

    def unpack(x):
        T_bc = rt_to_mat(Rotation.from_rotvec(x[2:5]).as_matrix(),
                         [x[0], x[1], cam_z])
        T_ot = rt_to_mat(Rotation.from_rotvec(x[8:11]).as_matrix(), x[5:8])
        return T_bc, T_ot

    def residuals(x):
        T_bc, T_ot = unpack(x)
        res = []
        for i in range(n):
            D = np.linalg.inv(T_ot) @ T_ob[i] @ T_bc @ T_ct[i]
            res.extend(D[:3, 3])
            res.extend(Rotation.from_matrix(D[:3, :3]).as_rotvec() * 0.1)
        return np.array(res)

    sol = least_squares(residuals, x0, method="lm", max_nfev=2000)
    T_bc, _ = unpack(sol.x)
    res = residuals(sol.x).reshape(n, 6)
    t_rms = float(np.sqrt((res[:, :3] ** 2).sum(1).mean())) * 1000
    r_rms = float(np.degrees(np.sqrt((res[:, 3:] ** 2).sum(1).mean()) / 0.1))
    print(f"residual RMS: {t_rms:.1f} mm / {r_rms:.2f}°")

    # base -> optical  =>  base -> camera_link (undo the optical convention)
    T_opt_link = np.eye(4)
    T_opt_link[:3, :3] = np.linalg.inv(M_LINK_FROM_OPT)
    T_bl = T_bc @ T_opt_link
    x, y, z = T_bl[:3, 3]
    roll, pitch, yaw = mat_to_rpy(T_bl)
    print("\n=== base_link -> camera_link ===")
    print(f"translation: x={x:.4f} y={y:.4f} z={z:.4f}  (old: 0.143 0 0.095)")
    print(f"rpy (rad):   roll={roll:.4f} pitch={pitch:.4f} yaw={yaw:.4f}")
    print(f"rpy (deg):   roll={math.degrees(roll):.2f} "
          f"pitch={math.degrees(pitch):.2f} yaw={math.degrees(yaw):.2f}")
    print("\nbase.launch.py static_transform_publisher arguments (x y z yaw pitch roll):")
    print(f"  ['{x:.4f}', '{y:.4f}', '{z:.4f}', "
          f"'{yaw:.4f}', '{pitch:.4f}', '{roll:.4f}', 'base_link', 'camera_link']")
    return t_rms, r_rms


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("url", nargs="?", help="rosbridge ws url")
    ap.add_argument("--tag-size", type=float, default=0.1175,
                    help="black square edge in meters (measure the screen!)")
    ap.add_argument("--cam-z", type=float, default=0.095,
                    help="ruler-measured camera height above base_link (m)")
    ap.add_argument("--out", default="/tmp/calib_run")
    ap.add_argument("--solve-only", metavar="DIR")
    ap.add_argument("--aire-path", default=AIRE_PATH_DEFAULT)
    args = ap.parse_args()

    if args.solve_only:
        solve(Path(args.solve_only), args.cam_z)
        return
    if not args.url:
        ap.error("url required unless --solve-only")
    out = Path(args.out)
    ok = asyncio.run(collect(args.url, args.aire_path, args.tag_size, out))
    if ok:
        solve(out, args.cam_z)
    else:
        print("collection incomplete; fix setup and rerun "
              f"(or --solve-only {out} if enough poses)")


if __name__ == "__main__":
    main()
