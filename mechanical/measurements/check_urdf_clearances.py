#!/usr/bin/env python3
"""Report zero-pose AABBs and sensor clearances from the generated URDF."""

from __future__ import annotations

import math
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def transform(xyz: str = "0 0 0", rpy: str = "0 0 0") -> np.ndarray:
    x, y, z = map(float, xyz.split())
    roll, pitch, yaw = map(float, rpy.split())
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array(((1, 0, 0), (0, cr, -sr), (0, sr, cr)))
    ry = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)))
    rz = np.array(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)))
    result = np.eye(4)
    result[:3, :3] = rz @ ry @ rx
    result[:3, 3] = (x, y, z)
    return result


def points_from_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    points = np.empty((count * 3, 3))
    for index in range(count):
        offset = 84 + index * 50 + 12
        for vertex in range(3):
            points[index * 3 + vertex] = struct.unpack_from("<3f", data, offset + vertex * 12)
    return points


def apply(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    return (matrix @ np.c_[points, np.ones(len(points))].T).T[:, :3]


def main() -> None:
    urdf = Path(sys.argv[1])
    repo = Path(__file__).resolve().parents[2]
    root = ET.parse(urdf).getroot()
    links = {link.get("name"): link for link in root.findall("link")}
    children = {}
    child_names = set()
    for joint in root.findall("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        origin = joint.find("origin")
        matrix = transform(
            origin.get("xyz", "0 0 0") if origin is not None else "0 0 0",
            origin.get("rpy", "0 0 0") if origin is not None else "0 0 0",
        )
        children.setdefault(parent, []).append((child, matrix))
        child_names.add(child)
    root_name = next(name for name in links if name not in child_names)
    world = {root_name: np.eye(4)}
    stack = [root_name]
    while stack:
        parent = stack.pop()
        for child, matrix in children.get(parent, []):
            world[child] = world[parent] @ matrix
            stack.append(child)

    bounds = {}
    for name, link in links.items():
        clouds = []
        for visual in link.findall("visual"):
            origin = visual.find("origin")
            local = transform(
                origin.get("xyz", "0 0 0") if origin is not None else "0 0 0",
                origin.get("rpy", "0 0 0") if origin is not None else "0 0 0",
            )
            geometry = visual.find("geometry")
            mesh = geometry.find("mesh")
            box = geometry.find("box")
            if mesh is not None:
                filename = mesh.get("filename")
                path = repo / "src/mentorpi_description" / filename.split("mentorpi_description/", 1)[1]
                points = points_from_stl(path)
                scale = np.array(list(map(float, mesh.get("scale", "1 1 1").split())))
                points *= scale
            elif box is not None:
                size = np.array(list(map(float, box.get("size").split()))) / 2
                points = np.array(
                    [[x, y, z] for x in (-size[0], size[0]) for y in (-size[1], size[1]) for z in (-size[2], size[2])]
                )
            else:
                continue
            clouds.append(apply(world[name] @ local, points))
        if clouds:
            points = np.vstack(clouds)
            bounds[name] = (points.min(axis=0), points.max(axis=0))

    for name in ("camera_link", "anker_prime_link", "so101_adapter_link", "lidar_riser_link"):
        lo, hi = bounds[name]
        print(name, "min_m", np.round(lo, 4).tolist(), "max_m", np.round(hi, 4).tolist())
    camera_lo, _ = bounds["camera_link"]
    _, battery_hi = bounds["anker_prime_link"]
    print("battery_to_camera_vertical_clearance_mm", round((camera_lo[2] - battery_hi[2]) * 1000, 1))
    laser_z = world["laser_frame"][2, 3]
    crossings = []
    for name, (lo, hi) in bounds.items():
        if name.startswith("so101_") and lo[2] <= laser_z <= hi[2]:
            crossings.append(name)
    print("laser_scan_z_m", round(float(laser_z), 4))
    print("so101_links_crossing_scan_plane", crossings)
    arm_max = max(hi[2] for name, (_, hi) in bounds.items() if name.startswith("so101_"))
    print("so101_zero_pose_max_z_m", round(float(arm_max), 4))


if __name__ == "__main__":
    main()
