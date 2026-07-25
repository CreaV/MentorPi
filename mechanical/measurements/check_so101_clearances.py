#!/usr/bin/env python3
"""Real-mesh clearance analysis for the SO-101 direct-mount lidar layout.

This design-decision check computes minimum vertex-cloud distances (not
AABBs) between the SO-101 links and the lidar head at zero pose and across
the full shoulder_pan sweep. With the recovered direct-mount lidar TF,
mount x=-0.155 has 36.2 mm worst-case head clearance over the full +/-110
degree sweep (2026-07-19 run, 5 degree step).

Usage:
  python check_so101_clearances.py <urdf> [--mount-dx 0.0 ...] [--pan-step 5]
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_urdf_clearances import apply, points_from_stl, transform  # noqa: E402

REPO = Path(__file__).resolve().parents[2]


CHECK_LINKS = ("so101_base_link", "so101_shoulder_link", "so101_upper_arm_link")


def load_model(urdf: Path):
    root = ET.parse(urdf).getroot()
    links = {l.get("name"): l for l in root.findall("link")}
    children, child_names, jmap = {}, set(), {}
    for j in root.findall("joint"):
        p, c = j.find("parent").get("link"), j.find("child").get("link")
        o = j.find("origin")
        m = transform(o.get("xyz", "0 0 0") if o is not None else "0 0 0",
                      o.get("rpy", "0 0 0") if o is not None else "0 0 0")
        children.setdefault(p, []).append((c, m))
        child_names.add(c)
        jmap[j.get("name")] = (p, c, m, j)
    root_name = next(n for n in links if n not in child_names)
    world = {root_name: np.eye(4)}
    stack = [root_name]
    while stack:
        p = stack.pop()
        for c, m in children.get(p, []):
            world[c] = world[p] @ m
            stack.append(c)
    return links, world, jmap


def link_cloud(links, name, max_pts=4000):
    pts_all = []
    for v in links[name].findall("visual"):
        o = v.find("origin")
        local = transform(o.get("xyz", "0 0 0") if o is not None else "0 0 0",
                          o.get("rpy", "0 0 0") if o is not None else "0 0 0")
        mesh = v.find("geometry").find("mesh")
        if mesh is None:
            continue
        path = REPO / "src/mentorpi_description" / mesh.get("filename").split("mentorpi_description/", 1)[1]
        pts = points_from_stl(path)
        pts = pts * np.array(list(map(float, mesh.get("scale", "1 1 1").split())))
        pts_all.append(apply(local, pts))
    pts = np.vstack(pts_all)
    if len(pts) > max_pts:
        pts = pts[np.random.default_rng(0).choice(len(pts), max_pts, replace=False)]
    return pts


def min_dist(a, b):
    best = 1e9
    for i in range(0, len(a), 400):
        best = min(best, np.linalg.norm(a[i:i + 400, None, :] - b[None, :, :], axis=2).min())
    return best



def rot_axis(axis, q):
    ax = np.asarray(axis, float)
    ax = ax / np.linalg.norm(ax)
    x, y, z = ax
    cq, sq, vq = math.cos(q), math.sin(q), 1 - math.cos(q)
    m = np.eye(4)
    m[:3, :3] = np.array([
        [cq + x * x * vq, x * y * vq - z * sq, x * z * vq + y * sq],
        [y * x * vq + z * sq, cq + y * y * vq, y * z * vq - x * sq],
        [z * x * vq - y * sq, z * y * vq + x * sq, cq + z * z * vq]])
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf", type=Path)
    ap.add_argument("--mount-dx", type=float, nargs="*", default=[0.0],
                    help="x offsets to add to the current so101 mount for what-if sweeps")
    ap.add_argument("--pan-step", type=int, default=5, help="pan sweep step in degrees")
    args = ap.parse_args()

    links, world, jmap = load_model(args.urdf)
    head_w = apply(world["laser_frame"], link_cloud(links, "laser_frame"))
    mount0 = world["so101_base_link"]

    clouds = {n: link_cloud(links, n) for n in CHECK_LINKS}
    rel = {n: np.linalg.inv(mount0) @ world[n] for n in CHECK_LINKS}

    pan_name = next(n for n, (p, c, _, _) in jmap.items() if c == "so101_shoulder_link")
    _, _, m_pan, jel = jmap[pan_name]
    axis = list(map(float, jel.find("axis").get("xyz").split()))
    lim = jel.find("limit")
    lo_a, hi_a = float(lim.get("lower")), float(lim.get("upper"))
    rel_pan = np.linalg.inv(mount0) @ world["so101_base_link"] @ m_pan

    for dx in args.mount_dx:
        mount = mount0.copy()
        mount[0, 3] += dx
        print(f"== mount x = {mount[0, 3]:+.4f} ==")
        print("  zero pose:")
        for n, cl in clouds.items():
            pw = apply(mount @ rel[n], cl)
            print(f"    {n:24s} head {min_dist(pw, head_w) * 1000:7.1f} mm")
        worst_h, arg_h = math.inf, 0
        sh = clouds["so101_shoulder_link"]
        for qd in range(int(math.degrees(lo_a)), int(math.degrees(hi_a)) + 1, args.pan_step):
            t = mount @ rel_pan @ rot_axis(axis, math.radians(qd))
            pw = apply(t, sh)
            dh = min_dist(pw, head_w)
            if dh < worst_h:
                worst_h, arg_h = dh, qd
        print(f"  pan sweep [{math.degrees(lo_a):.0f},{math.degrees(hi_a):.0f}]deg, shoulder:"
              f" worst head {worst_h * 1000:.1f} mm @{arg_h:+d}deg")


if __name__ == "__main__":
    main()
