#!/usr/bin/env python3
"""Rotation calibration: wheelbase+track_width (TODO Part 1 Step 1.2) and
gyro_z scale (Part 2 Step 2.4) in ONE experiment, laser wall as ground truth.

Setup: robot on flat ground, roughly FACING a flat wall 1-1.5 m away, at
least 0.5 m clearance all around (in-place rotation only; obstacle guard
stays on -- it never blocks rotation).

Procedure (all automatic over rosbridge):
  1. fit a line to the forward /scan sector -> wall direction (truth ref)
  2. rotate N full turns via motion/primitive (gyro-closed-loop), sampling
     wall direction + /odom yaw (cmd integral) + /odometry/filtered yaw
     (gyro integral) at every stop
  3. actual rotation = N*2pi + laser residual; report
       k_geom = odom_total / actual   -> (wheelbase+track_width) *= k_geom
       k_gyro = ekf_total  / actual   -> gyro_z scale = 1 / k_gyro
  4. repeat in the other direction (asymmetry = wheel/slip asymmetry)

Run (dev machine, aire-venv has numpy/websockets):
  python scripts/calibrate_rotation.py ws://192.168.8.117:9090 \
      --turns 2 --speed 0.5 --dir both
"""
from __future__ import annotations

import argparse
import asyncio
import math
import sys

import numpy as np

AIRE_PATH_DEFAULT = "/media/luo/Game/data/code/AIRE"
WHEELBASE = 0.1368      # base_node.py 当前值
TRACK_WIDTH = 0.1410


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def wrap_half_pi(a: float) -> float:
    """Wall line direction is mod pi; map delta to [-pi/2, pi/2)."""
    return (a + math.pi / 2) % math.pi - math.pi / 2


def yaw_from_quat(q: dict) -> float:
    siny = 2.0 * (q["w"] * q["z"] + q["x"] * q["y"])
    cosy = 1.0 - 2.0 * (q["y"] * q["y"] + q["z"] * q["z"])
    return math.atan2(siny, cosy)


def fit_wall(msg: dict, sector_deg: float = 40.0, max_range: float = 3.0):
    """Line-fit the forward scan sector. Returns (dir mod pi, n_inliers,
    rms_m, dist_m) or None."""
    a0 = float(msg["angle_min"])
    inc = float(msg["angle_increment"])
    pts = []
    for i, r in enumerate(msg["ranges"]):
        if r is None:
            continue
        r = float(r)
        if not math.isfinite(r) or not 0.15 < r < max_range:
            continue
        b = wrap_pi(a0 + i * inc)
        if abs(b) > math.radians(sector_deg):
            continue
        pts.append((r * math.cos(b), r * math.sin(b)))
    if len(pts) < 30:
        return None
    P = np.array(pts)
    # iterative PCA line fit with inlier rejection
    idx = np.arange(len(P))
    for _ in range(5):
        Q = P[idx]
        c = Q.mean(0)
        _, _, vt = np.linalg.svd(Q - c)
        d = vt[0]                       # line direction
        dist = np.abs((P - c) @ np.array([-d[1], d[0]]))
        new_idx = np.where(dist < 0.02)[0]
        if len(new_idx) < 30:
            break
        idx = new_idx
    Q = P[idx]
    c = Q.mean(0)
    _, s, vt = np.linalg.svd(Q - c)
    d = vt[0]
    rms = float(np.abs((Q - c) @ np.array([-d[1], d[0]])).std())
    ang = math.atan2(d[1], d[0]) % math.pi
    return ang, len(idx), rms, float(np.hypot(*c))


class Robot:
    def __init__(self, url: str, aire_path: str):
        sys.path.insert(0, aire_path)
        from air_engine.cloud.robot.rosbridge_client import RosbridgeClient
        from air_engine.cloud.robot.tools import RobotToolRouter
        self.client = RosbridgeClient(url)
        self.router = RobotToolRouter(self.client)

    async def _yaw(self, topic: str) -> float:
        msg = await self.client.subscribe_once(
            topic, "nav_msgs/msg/Odometry", timeout=5)
        return yaw_from_quat(msg["pose"]["pose"]["orientation"])

    async def settled_yaw(self, topic: str) -> float:
        """rosbridge delivers stale messages; double-read until consistent."""
        last = await self._yaw(topic)
        for _ in range(10):
            await asyncio.sleep(0.3)
            cur = await self._yaw(topic)
            if abs(wrap_pi(cur - last)) < 0.002:
                return cur
            last = cur
        print("  !! yaw did not settle, using last reading")
        return last

    async def wall(self):
        fits = []
        for _ in range(3):
            try:
                msg = await self.client.subscribe_once(
                    "/scan", "sensor_msgs/msg/LaserScan", timeout=6)
            except asyncio.TimeoutError:
                raise SystemExit(
                    "no /scan over rosbridge -- lidar running? (QoS: rosbridge "
                    "should auto-match Best Effort; check `ros2 topic hz /scan`)")
            f = fit_wall(msg)
            if f:
                fits.append(f)
            await asyncio.sleep(0.15)
        if not fits:
            return None
        # wall angles are mod pi -- average around the first sample
        ref = fits[0][0]
        ang = ref + np.mean([wrap_half_pi(f[0] - ref) for f in fits])
        n = int(np.mean([f[1] for f in fits]))
        rms = float(np.mean([f[2] for f in fits]))
        dist = float(np.mean([f[3] for f in fits]))
        return ang % math.pi, n, rms, dist

    async def rotate(self, deg: float, speed: float) -> bool:
        r = await self.router.handle_tool_call(
            "robot.rotate", {"angle_deg": deg, "speed": speed})
        if not r.get("success"):
            print(f"  !! rotate failed: {r}")
        return bool(r.get("success"))


async def sample(robot: Robot, label: str):
    w = await robot.wall()
    if w is None:
        raise SystemExit("wall fit failed -- is there a flat wall in the "
                         "forward sector within 3 m?")
    ang, n, rms, dist = w
    odom = await robot.settled_yaw("/odom")
    ekf = await robot.settled_yaw("/odometry/filtered")
    print(f"  [{label}] wall={math.degrees(ang):7.2f}°(mod180) "
          f"inliers={n} rms={rms*1000:.0f}mm dist={dist:.2f}m | "
          f"odom={math.degrees(odom):8.2f}° ekf={math.degrees(ekf):8.2f}°")
    if rms > 0.015:
        print("  !! wall fit noisy (>15mm) -- flat wall? clutter in sector?")
    return ang, odom, ekf


async def run_direction(robot: Robot, sign: int, turns: int, speed: float):
    name = "CCW(+)" if sign > 0 else "CW(-)"
    print(f"\n=== direction {name}, {turns} full turn(s) @ {speed} rad/s ===")
    wall0, odom0, ekf0 = await sample(robot, "start")
    actual = odom_tot = ekf_tot = 0.0
    wall_p, odom_p, ekf_p = wall0, odom0, ekf0
    for i in range(turns):
        if not await robot.rotate(sign * 360.0, speed):
            return None
        await asyncio.sleep(1.5)   # let the base fully stop
        wall_i, odom_i, ekf_i = await sample(robot, f"turn {i+1}")
        # laser truth: one commanded turn = sign*2pi + small residual.
        # wall rotates opposite to the robot in the laser frame.
        seg_actual = sign * 2 * math.pi - wrap_half_pi(wall_i - wall_p)
        seg_odom = sign * 2 * math.pi + wrap_pi(odom_i - odom_p - sign * 2 * math.pi)
        seg_ekf = sign * 2 * math.pi + wrap_pi(ekf_i - ekf_p - sign * 2 * math.pi)
        actual += seg_actual
        odom_tot += seg_odom
        ekf_tot += seg_ekf
        print(f"      seg: actual={math.degrees(seg_actual):8.2f}° "
              f"odom={math.degrees(seg_odom):8.2f}° "
              f"ekf={math.degrees(seg_ekf):8.2f}°")
        wall_p, odom_p, ekf_p = wall_i, odom_i, ekf_i
    return actual, odom_tot, ekf_tot


def report(results: list):
    actual = sum(r[0] for r in results)
    odom = sum(r[1] for r in results)
    ekf = sum(r[2] for r in results)
    print(f"\n===== totals over {len(results)} direction run(s) =====")
    print(f"actual (laser): {math.degrees(actual):9.2f}°")
    print(f"odom  (cmd):    {math.degrees(odom):9.2f}°")
    print(f"ekf   (gyro):   {math.degrees(ekf):9.2f}°")
    k_geom = odom / actual
    k_gyro = ekf / actual
    wt_old = WHEELBASE + TRACK_WIDTH
    wt_new = wt_old * k_geom
    print(f"\nStep 1.2  k_geom = odom/actual = {k_geom:.4f}")
    print(f"  wheelbase+track_width: {wt_old:.4f} -> {wt_new:.4f}")
    print(f"  (等比分配: wheelbase {WHEELBASE:.4f} -> {WHEELBASE*k_geom:.4f}, "
          f"track_width {TRACK_WIDTH:.4f} -> {TRACK_WIDTH*k_geom:.4f}; "
          f"改 base_node.py 的 _mecanum 常量)")
    print(f"\nStep 2.4  k_gyro = ekf/actual = {k_gyro:.4f}")
    if abs(k_gyro - 1.0) < 0.005:
        print("  gyro_z 比例误差 <0.5%, 无需修正")
    else:
        print(f"  gyro_z 需乘 {1.0/k_gyro:.4f} (base_node 需新增比例参数)")
    if len(results) == 2:
        k1 = results[0][1] / results[0][0]
        k2 = results[1][1] / results[1][0]
        print(f"\n方向对称性: k_geom CCW={k1:.4f} vs CW={k2:.4f} "
              f"(差 >2% 提示左右轮不对称, 见 UMBmark)")


async def main_async(args):
    robot = Robot(args.url, args.aire_path)
    await robot.client.connect()
    try:
        results = []
        dirs = {"ccw": [1], "cw": [-1], "both": [1, -1]}[args.dir]
        for sign in dirs:
            r = await run_direction(robot, sign, args.turns, args.speed)
            if r is None:
                print("aborted")
                return
            results.append(r)
        report(results)
    finally:
        await robot.client.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("url", help="ws://<robot-ip>:9090")
    ap.add_argument("--turns", type=int, default=2, help="full turns per direction")
    ap.add_argument("--speed", type=float, default=0.5, help="rad/s (slow = less slip)")
    ap.add_argument("--dir", choices=["ccw", "cw", "both"], default="both")
    ap.add_argument("--aire-path", default=AIRE_PATH_DEFAULT)
    asyncio.run(main_async(ap.parse_args()))


if __name__ == "__main__":
    main()
