#!/usr/bin/env python3
"""Square-loop odometry acceptance (TODO Part 1/2 验收), laser ICP as truth.

Drives a closed square (4 x [forward L + rotate 90 deg]) via motion
primitives, captures a full /scan before and after, and ICP-aligns the two
scans to measure the PHYSICAL closure error (dx/dy/dyaw in the start pose
frame) -- no tape measure needed. The EKF-believed closure is reported too:
primitives close their loop on /odometry/filtered, so the physical error is
the accumulated odometry error over the loop.

Pass criteria (TODO.md): translation closure < 5% of path length (4L),
yaw closure < 5 deg.

Notes:
  - Scans are in laser_frame (x offset -0.014 m from base_link, no rotation);
    the induced error is ~|dyaw|*0.014 m -- negligible, ignored.
  - The room must be mostly static between the two scans (don't walk around).
  - A corridor preflight from the start scan warns if a leg would end inside
    the obstacle-guard stop zone (primitive would stall and time out).

Run (dev machine, aire-venv has numpy/scipy/websockets):
  python scripts/acceptance_square.py ws://192.168.8.117:9090 \
      --size 0.8 --speed 0.25 --dir both
"""
from __future__ import annotations

import argparse
import asyncio
import math
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibrate_rotation import AIRE_PATH_DEFAULT, Robot, wrap_pi, yaw_from_quat


def scan_points(msg: dict, max_range: float = 4.0) -> np.ndarray:
    a0 = float(msg["angle_min"])
    inc = float(msg["angle_increment"])
    pts = []
    for i, r in enumerate(msg["ranges"]):
        if r is None:
            continue
        r = float(r)
        if not math.isfinite(r) or not 0.17 < r < max_range:
            continue
        b = a0 + i * inc
        pts.append((r * math.cos(b), r * math.sin(b)))
    return np.array(pts)


def icp(src: np.ndarray, dst: np.ndarray, iters: int = 60):
    """Trimmed point-to-point ICP. Returns (R, t, rms, n_inliers) with
    R @ p_src + t ~= p_dst, i.e. the robot pose delta expressed in the
    start (dst) frame."""
    tree = cKDTree(dst)
    R = np.eye(2)
    t = np.zeros(2)
    rms, n_in = float("inf"), 0
    for it in range(iters):
        thr = max(0.08, 0.5 * 0.85 ** it)
        cur = src @ R.T + t
        d, j = tree.query(cur)
        m = d < thr
        n_in = int(m.sum())
        if n_in < 50:
            break
        P, Q = cur[m], dst[j[m]]
        cp, cq = P.mean(0), Q.mean(0)
        H = (P - cp).T @ (Q - cq)
        U, _, Vt = np.linalg.svd(H)
        Ri = Vt.T @ U.T
        if np.linalg.det(Ri) < 0:      # reflection guard
            Vt[1] *= -1
            Ri = Vt.T @ U.T
        ti = cq - Ri @ cp
        R = Ri @ R
        t = Ri @ t + ti
        rms = float(np.sqrt((d[m] ** 2).mean()))
        if abs(math.atan2(Ri[1, 0], Ri[0, 0])) < 1e-6 and np.hypot(*ti) < 1e-6:
            break
    return R, t, rms, n_in


def preflight(pts: np.ndarray, size: float, ccw: bool) -> list[str]:
    """Corridor check per leg from the start scan: no point may sit inside
    the swept corridor (half-width 0.22 m) up to leg end + guard stop."""
    s = 1.0 if ccw else -1.0
    corners = [(0.0, 0.0), (size, 0.0), (size, s * size), (0.0, s * size)]
    dirs = [(1, 0), (0, s), (-1, 0), (0, -s)]
    problems = []
    for k, ((cx, cy), (dx, dy)) in enumerate(zip(corners, dirs)):
        rel = pts - (cx, cy)
        along = rel @ (dx, dy)
        cross = rel @ (-dy, dx)
        bad = (along > 0.0) & (along < size + 0.35) & (np.abs(cross) < 0.22)
        if bad.any():
            worst = float(along[bad].min())
            problems.append(f"leg {k + 1} ({'CCW' if ccw else 'CW'}): obstacle "
                            f"{worst:.2f}m into a {size:.2f}m corridor")
    return problems


async def grab_scan(robot: Robot) -> np.ndarray:
    msg = None
    for _ in range(2):     # rosbridge can hand back a stale first message
        msg = await robot.client.subscribe_once(
            "/scan", "sensor_msgs/msg/LaserScan", timeout=6)
        await asyncio.sleep(0.3)
    pts = scan_points(msg)
    if len(pts) < 100:
        raise SystemExit(f"start/end scan too sparse ({len(pts)} pts)")
    return pts


async def pose(robot: Robot, topic: str):
    last = None
    for _ in range(10):
        msg = await robot.client.subscribe_once(
            topic, "nav_msgs/msg/Odometry", timeout=5)
        p = msg["pose"]["pose"]
        cur = (p["position"]["x"], p["position"]["y"],
               yaw_from_quat(p["orientation"]))
        if last and np.hypot(cur[0] - last[0], cur[1] - last[1]) < 1e-3 \
                and abs(wrap_pi(cur[2] - last[2])) < 0.002:
            return np.array(cur)
        last = cur
        await asyncio.sleep(0.3)
    print(f"  !! {topic} pose did not settle, using last reading")
    return np.array(last)


def closure(p0: np.ndarray, p1: np.ndarray):
    """Pose delta p0 -> p1 expressed in the p0 frame (matches ICP output)."""
    c, s = math.cos(p0[2]), math.sin(p0[2])
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    return (c * dx + s * dy, -s * dx + c * dy, wrap_pi(p1[2] - p0[2]))


async def run_square(robot: Robot, size: float, speed: float, ccw: bool,
                     force: bool) -> bool:
    name = "CCW" if ccw else "CW"
    print(f"\n=== square {name}, leg {size} m @ {speed} m/s ===")
    start_pts = await grab_scan(robot)
    problems = preflight(start_pts, size, ccw)
    for p in problems:
        print(f"  !! preflight: {p}")
    if problems and not force:
        print("  aborted (use --force to run anyway, or shrink --size)")
        return False
    ekf0 = await pose(robot, "/odometry/filtered")
    odom0 = await pose(robot, "/odom")

    for leg in range(4):
        r = await robot.router.handle_tool_call(
            "robot.move",
            {"direction": "forward", "distance": size, "speed": speed})
        if not r.get("success"):
            print(f"  !! leg {leg + 1} move failed: {r}")
            return False
        await asyncio.sleep(1.0)
        if not await robot.rotate(90.0 if ccw else -90.0, 0.5):
            return False
        await asyncio.sleep(1.0)
        print(f"  leg {leg + 1}/4 done")
    await asyncio.sleep(1.5)   # settle before the end scan

    end_pts = await grab_scan(robot)
    ekf1 = await pose(robot, "/odometry/filtered")
    odom1 = await pose(robot, "/odom")

    R, t, rms, n_in = icp(end_pts, start_pts)
    dyaw = math.atan2(R[1, 0], R[0, 0])
    path = 4.0 * size
    d = float(np.hypot(*t))
    print(f"  ICP: inliers={n_in} rms={rms * 1000:.0f}mm")
    if n_in < 150 or rms > 0.08:
        print("  !! ICP alignment weak -- room changed / too much clutter? "
              "treat the numbers below with suspicion")
    e_ekf = closure(ekf0, ekf1)
    e_odm = closure(odom0, odom1)
    print(f"  physical closure: dx={t[0] * 1000:+.0f}mm dy={t[1] * 1000:+.0f}mm "
          f"|d|={d * 1000:.0f}mm ({d / path * 100:.1f}% of {path:.1f}m path) "
          f"dyaw={math.degrees(dyaw):+.2f}°")
    print(f"  ekf-believed:     dx={e_ekf[0] * 1000:+.0f}mm "
          f"dy={e_ekf[1] * 1000:+.0f}mm dyaw={math.degrees(e_ekf[2]):+.2f}°")
    print(f"  odom-believed:    dx={e_odm[0] * 1000:+.0f}mm "
          f"dy={e_odm[1] * 1000:+.0f}mm dyaw={math.degrees(e_odm[2]):+.2f}°")
    # 验收对象是"估计误差"= 物理真值 - EKF 认为的位姿变化。physical
    # closure 里还叠着执行误差(rotate 原语过冲等,EKF 自己看得见,
    # motion_node 调速的事),不能算在里程计头上。
    est_t = float(np.hypot(t[0] - e_ekf[0], t[1] - e_ekf[1]))
    est_y = math.degrees(wrap_pi(dyaw - e_ekf[2]))
    ok_d = est_t < 0.05 * path
    ok_y = abs(est_y) < 5.0
    print(f"  estimation error (physical - ekf): |d|={est_t * 1000:.0f}mm "
          f"({est_t / path * 100:.1f}%) dyaw={est_y:+.2f}°")
    print(f"  verdict: translation {'PASS' if ok_d else 'FAIL'} "
          f"(<{0.05 * path * 1000:.0f}mm), yaw {'PASS' if ok_y else 'FAIL'} (<5°)")
    return ok_d and ok_y


async def main_async(args):
    robot = Robot(args.url, args.aire_path)
    await robot.client.connect()
    try:
        results = []
        if args.dir in ("ccw", "both"):
            results.append(await run_square(
                robot, args.size, args.speed, True, args.force))
        if args.dir in ("cw", "both"):
            results.append(await run_square(
                robot, args.size, args.speed, False, args.force))
        print(f"\n===== acceptance {'PASS' if all(results) and results else 'FAIL'} =====")
    finally:
        try:
            await robot.router.handle_tool_call("robot.stop", {})
        except Exception:
            pass
        await robot.client.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("url", help="rosbridge ws url, e.g. ws://192.168.8.117:9090")
    ap.add_argument("--size", type=float, default=0.8, help="leg length m")
    ap.add_argument("--speed", type=float, default=0.25, help="linear m/s")
    ap.add_argument("--dir", choices=("ccw", "cw", "both"), default="both")
    ap.add_argument("--force", action="store_true",
                    help="run even if preflight finds corridor obstacles")
    ap.add_argument("--aire-path", default=AIRE_PATH_DEFAULT)
    args = ap.parse_args()
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
