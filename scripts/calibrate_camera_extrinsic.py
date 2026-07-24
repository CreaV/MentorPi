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
  4. print the base_link -> camera_link translation+RPY for camera_joint
  5. optionally update mecanum.xacro atomically with --update-xacro

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
import re
import sys
import xml.etree.ElementTree as ET
import time
from pathlib import Path

import numpy as np

AIRE_PATH_DEFAULT = "/media/luo/Game/data/code/AIRE"
DEFAULT_XACRO = (Path(__file__).resolve().parents[1] /
                 "src/mentorpi_description/urdf/mecanum.xacro")


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

    async def _odom_raw(self, topic="/odometry/filtered"):
        msg = await self.client.subscribe_once(
            topic, "nav_msgs/msg/Odometry", timeout=5)
        p = msg["pose"]["pose"]["position"]
        q = msg["pose"]["pose"]["orientation"]
        return quat_xyzw_to_mat([q["x"], q["y"], q["z"], q["w"]],
                                [p["x"], p["y"], p["z"]])

    async def odom(self, topic="/odometry/filtered"):
        """rosbridge 的 subscribe 会交付陈旧/乱序消息 (实测停车后位姿
        "倒退" 6cm = 读到了运动中途的旧帧)。机器人采样时必然静止, 连读
        两次间隔 0.3s、位姿一致才采信 —— 一致即新鲜。"""
        last = await self._odom_raw(topic)
        for _ in range(8):
            await asyncio.sleep(0.3)
            cur = await self._odom_raw(topic)
            if (np.linalg.norm(cur[:3, 3] - last[:3, 3]) < 0.002
                    and np.abs(cur[:3, :3] - last[:3, :3]).max() < 0.005):
                return cur
            last = cur
        print("  odom did not settle — using last reading")
        return last

    async def set_guard(self, enabled: bool) -> None:
        try:
            await self.client.call_service(
                "/mentorpi_base/set_parameters", "rcl_interfaces/srv/SetParameters",
                {"parameters": [{"name": "obstacle_guard",
                                 "value": {"type": 1, "bool_value": enabled}}]},
                timeout=5.0)
            print(f"obstacle guard -> {enabled}")
        except Exception as exc:  # noqa: BLE001
            print(f"!! failed to set obstacle_guard={enabled}: {exc}")

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
        if not await robot.rotate(ang):
            print("  aim rotate failed (guard/hardware) — keeping current heading")
            return r
    await asyncio.sleep(1.0)
    return det.detect(await robot.frame())


async def health_check(robot) -> bool:
    """Prove the drivetrain physically moves: 15° there and back, verified
    against EKF yaw (gyro). Catches dead STM32 / power switch off — those
    fail silently otherwise (cmd-integrated odom moves, robot doesn't)."""
    def yaw_of(T):
        return math.atan2(T[1, 0], T[0, 0])
    y0 = yaw_of(await robot.odom())
    ok = await robot.rotate(15)
    y1 = yaw_of(await robot.odom())
    moved = math.degrees(abs(math.atan2(math.sin(y1 - y0), math.cos(y1 - y0))))
    if ok and moved >= 7.0:
        await robot.rotate(-15)
        return True
    print(f"!! drivetrain health check FAILED (rotate ok={ok}, gyro moved "
          f"{moved:.1f}°) — check STM32 power switch / battery / USB serial")
    return False


async def capture(robot, det, out: Path, records: list, note: str):
    await asyncio.sleep(1.2)  # settle + let the 2Hz viewer frame refresh
    T_odom_base = await robot.odom()
    T_raw_base = await robot.odom("/odom")
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
                    "T_raw_base": T_raw_base.tolist(),
                    "T_cam_tag": T_cam_tag.tolist()})
    print(f"  [{note}] captured #{idx} (tag u={u:.0f}px)")


async def collect(url: str, aire_path: str, tag_size: float, out: Path,
                  intrinsics=None, far_dist: float = 1.6, near_dist: float = 1.0):
    robot = Robot(url, aire_path)
    if intrinsics is not None:
        fx, fy, cx, cy = intrinsics
    else:
        fx, fy, cx, cy = await robot.camera_info()
    print(f"camera: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")
    det = TagDetector(fx, fy, cx, cy, tag_size)

    if not await health_check(robot):
        return False

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

    # -- stations: lateral sweep at two depths --
    # At each station: (re)aim, capture at -10°/0°/+10° headings.
    async def reacquire(hint_deg):
        """Sweep toward the expected tag bearing until detected."""
        for ang in (hint_deg, -2 * hint_deg, 3 * hint_deg):
            if ang == 0 or not await robot.rotate(ang):
                return None
            await asyncio.sleep(1.0)
            if det.detect(await robot.frame()) is not None:
                return True
        return None

    async def station(label, hint_deg=0):
        r = await aim_tag(robot, det, fx, cx)
        if r is None and hint_deg and await reacquire(hint_deg):
            r = await aim_tag(robot, det, fx, cx)
        if r is None:
            print(f"  [{label}] tag lost — station skipped")
            return
        await capture(robot, det, out, records, f"{label} aim")
        # 宽弧多朝向: ±22°/±11°。小弧 (±10°) 下 (相机y, 相机yaw) 在解空间
        # 里近简并 (两数据集分别解出 +6°/-6° yaw 而残差不变), 大弧才能把
        # 杠杆臂的弯曲信息压进数据。
        moved = 0.0
        for target in (-22, -11, 11, 22):
            d = target - moved
            if await robot.rotate(d):
                moved = target
                await capture(robot, det, out, records, f"{label} {target:+d}°")
            else:
                print(f"  [{label}] rotate {d}° failed — heading skipped")
        await robot.rotate(-moved)  # back to aim

    # v3 站位: 弧站 + 直线站 (见 solve_v3 docstring)。
    # 直线段用 tag 实测距离规划: 站位全部保持在 tag 1.0m 以外 —— 守卫
    # 全程开启也不会进减速区 (慢行区 <0.75m), 更不可能撞上平板
    # (2026-07-12 曾因关守卫 + 固定步数前进撞倒平板, 勿回退)。
    async def tag_dist():
        await asyncio.sleep(1.0)
        r = det.detect(await robot.frame())
        return None if r is None else float(np.linalg.norm(np.array(r[0])[:3, 3]))

    await station("S0 arc")
    # 后退拉大基线到 far_dist (或被守卫/失检拦住)
    d = await tag_dist()
    while d is not None and d < far_dist:
        if not await robot.move("backward", 0.25):
            print(f"  rear blocked at tag dist {d:.2f}m")
            break
        d = await tag_dist()
    await capture(robot, det, out, records, "L0 line")
    k = 1
    while k <= 6:
        d = await tag_dist()
        if d is None:
            print("  line: tag lost, stopping")
            break
        if d < near_dist:  # 站位下限: 下一步后仍 >0.75m
            print(f"  line: reached closest station (tag {d:.2f}m)")
            break
        if not await robot.move("forward", 0.25):
            break
        await capture(robot, det, out, records, f"L{k} line")
        k += 1

    (out / "records.json").write_text(json.dumps(records, indent=1))
    print(f"collected {len(records)} poses -> {out}/records.json")
    await robot.client.close()
    return len(records) >= 8


# ---------- solve ----------

def solve_v3(out: Path, cam_z: float):
    """Arc + line model.

    Arc records ("S0 arc ..."): in-place rotations at a fixed point p0
    (EKF yaw per pose is gyro-accurate) — pins the camera lever arm and
    the base rotation axis in camera frame (pitch/roll).
    Line records ("Lk line"): pure straight driving, no rotations. The
    RAW cmd-integrated odometry arc length is wheel-calibrated and
    trustworthy on a straight constant-speed run (unlike the EKF pose,
    which lags at low speed, and unlike strafe, which slips) — this
    metric baseline pins the camera yaw, which arc data alone cannot
    (camera-yaw/tag-yaw gauge freedom). Line direction is a free unknown.
    """
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation

    records = json.loads((out / "records.json").read_text())
    arc = [r for r in records if r["note"].startswith("S")]
    line = [r for r in records if r["note"].startswith("L")]
    if not arc or len(line) < 2:
        print(f"v3 needs arc + >=3 line records (have {len(arc)}/{len(line)})")
        return None
    print(f"solving v3 with {len(arc)} arc + {len(line)} line poses "
          f"(camera z fixed at {cam_z} m)")

    def yaw_of(T):
        return math.atan2(T[1][0], T[0][0])

    T_ct = [np.array(r["T_cam_tag"]) for r in records]
    yaws = [yaw_of(r["T_odom_base"]) for r in records]
    p0 = np.array(arc[0]["T_odom_base"])[:2, 3]           # arc anchor (gauge)
    raw0 = np.array(line[0]["T_raw_base"])[:2, 3]
    s = [float(np.linalg.norm(np.array(r["T_raw_base"])[:2, 3] - raw0))
         for r in line]
    # initial line direction from EKF endpoints (direction survives lag)
    e0 = np.array(line[0]["T_odom_base"])[:2, 3]
    e1 = np.array(line[-1]["T_odom_base"])[:2, 3]
    theta0 = math.atan2(*(e1 - e0)[::-1]) if np.linalg.norm(e1 - e0) > 1e-3 else 0.0

    # tag 只当"3D 点"用: 平面 tag 近正对时 PnP 的旋转解有几度级抖动/
    # 双解偏置 (历次 yaw 在 ±7° 乱跳的元凶), 平移则稳到 ~0.5% 距离。
    # 位置-only 残差把旋转噪声整体挡在门外。
    R0 = M_LINK_FROM_OPT
    x0 = np.zeros(11)
    x0[0:2] = [0.143, 0.0]
    x0[2:5] = Rotation.from_matrix(R0).as_rotvec()
    T_bc0 = rt_to_mat(R0, [0.143, 0.0, cam_z])
    T_ot0 = np.array(arc[0]["T_odom_base"]) @ T_bc0 @ np.array(arc[0]["T_cam_tag"])
    x0[5:8] = T_ot0[:3, 3]          # tag 位置 (点)
    x0[8:10] = e0                   # 直线起点
    x0[10] = theta0                 # 直线方向

    n_arc = len(arc)
    tag_pos_cam = [T[:3, 3] for T in T_ct]

    def residuals(x):
        T_bc = rt_to_mat(Rotation.from_rotvec(x[2:5]).as_matrix(),
                         [x[0], x[1], cam_z])
        p_tag = x[5:8]
        q0, th = x[8:10], x[10]
        res = []
        for i in range(len(records)):
            if i < n_arc:
                pos = p0
            else:
                k = i - n_arc
                pos = q0 + s[k] * np.array([math.cos(th), math.sin(th)])
            T_base = rt_to_mat(Rotation.from_euler('z', yaws[i]).as_matrix(),
                               [pos[0], pos[1], 0.0])
            pred = (T_base @ T_bc)[:3, :3] @ tag_pos_cam[i] + (T_base @ T_bc)[:3, 3]
            res.extend(pred - p_tag)
        return np.array(res)

    sol = least_squares(residuals, x0, method="lm", max_nfev=5000)
    T_bc = rt_to_mat(Rotation.from_rotvec(sol.x[2:5]).as_matrix(),
                     [sol.x[0], sol.x[1], cam_z])
    res = residuals(sol.x).reshape(len(records), 3)
    t_rms = float(np.sqrt((res ** 2).sum(1).mean())) * 1000
    print(f"residual RMS: {t_rms:.1f} mm (position-only)")
    extrinsic = _print_extrinsic(T_bc)
    return t_rms, None, extrinsic


def _print_extrinsic(T_bc):
    T_opt_link = np.eye(4)
    T_opt_link[:3, :3] = np.linalg.inv(M_LINK_FROM_OPT)
    T_bl = T_bc @ T_opt_link
    x, y, z = T_bl[:3, 3]
    roll, pitch, yaw = mat_to_rpy(T_bl)
    print("\n=== base_link -> camera_link ===")
    print(f"translation: x={x:.4f} y={y:.4f} z={z:.4f}")
    print(f"rpy (rad):   roll={roll:.4f} pitch={pitch:.4f} yaw={yaw:.4f}")
    print(f"rpy (deg):   roll={math.degrees(roll):.2f} "
          f"pitch={math.degrees(pitch):.2f} yaw={math.degrees(yaw):.2f}")
    print("\nmecanum.xacro camera_joint:")
    print(f"  <origin xyz=\"{x:.4f} {y:.4f} {z:.4f}\" "
          f"rpy=\"{roll:.4f} {pitch:.4f} {yaw:.4f}\"/>")
    return tuple(float(v) for v in (x, y, z, roll, pitch, yaw))


def solve(out: Path, cam_z: float, trust_odom: bool = False):
    """Rotation-only hand-eye: trust per-station in-place rotations (EKF yaw
    = bias-corrected gyro, reliable) and treat every station's 2D position
    as a free unknown. Command-integrated odometry TRANSLATION is open-loop
    on a slipping mecanum base and measured 4x off vs the tag — it must not
    constrain the solve. In-place rotations alone observe the camera lever
    arm (x, y), yaw, and the base rotation axis seen in the camera (pitch,
    roll); camera z stays ruler-fixed."""
    records = json.loads((out / "records.json").read_text())
    if any(r["note"].startswith("L") for r in records):
        return solve_v3(out, cam_z)

    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation

    T_ob = [np.array(r["T_odom_base"]) for r in records]
    T_ct = [np.array(r["T_cam_tag"]) for r in records]
    station_names = sorted({r["note"].split()[0] for r in records})
    st_idx = [station_names.index(r["note"].split()[0]) for r in records]
    yaws = [math.atan2(T[1, 0], T[0, 0]) for T in T_ob]
    n, ns = len(records), len(station_names)
    print(f"solving with {n} poses at {ns} stations "
          f"(camera z fixed at {cam_z} m; odom translation "
          f"{'TRUSTED (forward-line pattern)' if trust_odom else 'NOT trusted'})")

    # params: base->cam_optical [x y rvec(3)] + odom->tag [xyz rvec(3)]
    #         + per-station 2D position (station 0 pinned at its odom value)
    R0 = M_LINK_FROM_OPT  # camera level, no mount rotation
    x0 = np.zeros(11 + 2 * (ns - 1))
    x0[0:2] = [0.143, 0.0]
    x0[2:5] = Rotation.from_matrix(R0).as_rotvec()
    T_bc0 = rt_to_mat(R0, [0.143, 0.0, cam_z])
    T_ot0 = T_ob[0] @ T_bc0 @ T_ct[0]
    x0[5:8] = T_ot0[:3, 3]
    x0[8:11] = Rotation.from_matrix(T_ot0[:3, :3]).as_rotvec()
    st_odom = []
    for s in range(ns):
        i = st_idx.index(s)
        st_odom.append(T_ob[i][:2, 3])
        if s > 0:
            x0[11 + 2 * (s - 1): 13 + 2 * (s - 1)] = st_odom[s]

    def station_pos(x, s):
        if trust_odom or s == 0:
            return st_odom[s]
        return x[11 + 2 * (s - 1): 13 + 2 * (s - 1)]

    def unpack(x):
        T_bc = rt_to_mat(Rotation.from_rotvec(x[2:5]).as_matrix(),
                         [x[0], x[1], cam_z])
        T_ot = rt_to_mat(Rotation.from_rotvec(x[8:11]).as_matrix(), x[5:8])
        return T_bc, T_ot

    def residuals(x):
        T_bc, T_ot = unpack(x)
        res = []
        for i in range(n):
            p = station_pos(x, st_idx[i])
            T_base = rt_to_mat(Rotation.from_euler('z', yaws[i]).as_matrix(),
                               [p[0], p[1], 0.0])
            D = np.linalg.inv(T_ot) @ T_base @ T_bc @ T_ct[i]
            res.extend(D[:3, 3])
            res.extend(Rotation.from_matrix(D[:3, :3]).as_rotvec() * 0.1)
        return np.array(res)

    sol = least_squares(residuals, x0, method="lm", max_nfev=5000)
    T_bc, _ = unpack(sol.x)
    res = residuals(sol.x).reshape(n, 6)
    t_rms = float(np.sqrt((res[:, :3] ** 2).sum(1).mean())) * 1000
    r_rms = float(np.degrees(np.sqrt((res[:, 3:] ** 2).sum(1).mean()) / 0.1))
    print(f"residual RMS: {t_rms:.1f} mm / {r_rms:.2f}°")

    extrinsic = _print_extrinsic(T_bc)
    return t_rms, r_rms, extrinsic


def update_camera_joint(xacro_path: Path, extrinsic: tuple) -> None:
    """Atomically replace only camera_joint origin in the xacro."""
    x, y, z, roll, pitch, yaw = extrinsic
    text = xacro_path.read_text()
    joint_re = re.compile(
        r'(<joint name="camera_joint" type="fixed">.*?'
        r'<origin xyz=")[^"]+(" rpy=")[^"]+("/>.*?</joint>)',
        re.DOTALL,
    )
    replacement = (
        rf'\g<1>{x:.4f} {y:.4f} {z:.4f}'
        rf'\g<2>{roll:.4f} {pitch:.4f} {yaw:.4f}\g<3>'
    )
    updated, count = joint_re.subn(replacement, text)
    if count != 1:
        raise RuntimeError(
            f"expected exactly one camera_joint origin in {xacro_path}; "
            f"found {count}"
        )
    # keep the provenance comment's date current if present
    updated = re.sub(
        r"(AprilTag hand-eye calibration, )\d{4}-\d{2}-\d{2}",
        rf"\g<1>{time.strftime('%Y-%m-%d')}",
        updated, count=1,
    )
    ET.fromstring(updated)
    tmp = xacro_path.with_suffix(xacro_path.suffix + ".tmp")
    tmp.write_text(updated)
    tmp.replace(xacro_path)
    print(f"updated {xacro_path}")


def apply_result(result, args) -> None:
    if result is None or not args.update_xacro:
        return
    t_rms, _r_rms, extrinsic = result
    if t_rms > args.max_update_rms_mm:
        raise RuntimeError(
            f"refusing xacro update: residual {t_rms:.1f} mm exceeds "
            f"--max-update-rms-mm={args.max_update_rms_mm:.1f}"
        )
    update_camera_joint(args.xacro_path, extrinsic)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("url", nargs="?", help="rosbridge ws url")
    ap.add_argument("--tag-size", type=float, default=0.1175,
                    help="black square edge in meters (measure the screen!)")
    ap.add_argument("--cam-z", type=float, default=0.095,
                    help="ruler-measured camera height above base_link (m)")
    ap.add_argument("--out", default="/tmp/calib_run")
    ap.add_argument("--far-dist", type=float, default=1.6,
                    help="line-station far tag distance (m); lower for tight rooms")
    ap.add_argument("--near-dist", type=float, default=1.0,
                    help="closest line-station tag distance (m); never below tablet safety")
    ap.add_argument("--solve-only", metavar="DIR")
    ap.add_argument("--update-xacro", action="store_true",
                    help="atomically update camera_joint after a good solve")
    ap.add_argument("--xacro-path", type=Path, default=DEFAULT_XACRO,
                    help="xacro to update (default: repository mecanum.xacro)")
    ap.add_argument("--max-update-rms-mm", type=float, default=20.0,
                    help="refuse --update-xacro above this position RMS")
    ap.add_argument("--trust-odom", action="store_true",
                    help="trust odom station positions (forward-line pattern)")
    ap.add_argument("--aire-path", default=AIRE_PATH_DEFAULT)
    ap.add_argument("--intrinsics", nargs=4, type=float,
                    metavar=("FX", "FY", "CX", "CY"),
                    help="bypass rosbridge camera_info (its QoS is flaky)")
    args = ap.parse_args()

    if args.solve_only:
        result = solve(Path(args.solve_only), args.cam_z, args.trust_odom)
        apply_result(result, args)
        return
    if not args.url:
        ap.error("url required unless --solve-only")
    out = Path(args.out)
    ok = asyncio.run(collect(args.url, args.aire_path, args.tag_size, out,
                             intrinsics=args.intrinsics,
                             far_dist=args.far_dist, near_dist=args.near_dist))
    if ok:
        result = solve(out, args.cam_z, trust_odom=True)
        apply_result(result, args)
    else:
        print("collection incomplete; fix setup and rerun "
              f"(or --solve-only {out} if enough poses)")


if __name__ == "__main__":
    main()
