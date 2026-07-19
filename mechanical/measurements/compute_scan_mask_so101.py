#!/usr/bin/env python3
"""Compute the SO-101 rear LaserScan mask from URDF mesh/plane intersections.

The calculation uses the generated zero-pose URDF. It intersects every
SO-101 visual-mesh triangle with the horizontal laser_frame plane, converts
the intersection points to polar angles about laser_frame, and reports the
rear-sector half width plus a configurable safety margin.

Usage:
  python compute_scan_mask_so101.py robot.urdf [--margin-deg 5]
"""

from __future__ import annotations

import argparse
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from check_urdf_clearances import apply, transform


REPO = Path(__file__).resolve().parents[2]


def triangles_from_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    triangles = np.empty((count, 3, 3), dtype=float)
    for index in range(count):
        offset = 84 + index * 50 + 12
        for vertex in range(3):
            triangles[index, vertex] = struct.unpack_from(
                "<3f", data, offset + vertex * 12
            )
    return triangles


def mesh_path(urdf: Path, filename: str) -> Path:
    marker = "mentorpi_description/"
    if marker in filename:
        return REPO / "src/mentorpi_description" / filename.split(marker, 1)[1]
    return (urdf.parent / filename).resolve()


def load_world(root: ET.Element) -> dict[str, np.ndarray]:
    links = {link.get("name") for link in root.findall("link")}
    children: dict[str, list[tuple[str, np.ndarray]]] = {}
    child_names: set[str] = set()
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
    return world


def triangle_plane_points(triangles: np.ndarray, plane_z: float) -> np.ndarray:
    spans = (triangles[:, :, 2].min(axis=1) <= plane_z) & (
        triangles[:, :, 2].max(axis=1) >= plane_z
    )
    selected = triangles[spans]
    points: list[np.ndarray] = []
    for first, second in ((0, 1), (1, 2), (2, 0)):
        a = selected[:, first]
        b = selected[:, second]
        dz = b[:, 2] - a[:, 2]
        crossing = (np.abs(dz) > 1e-12) & (
            ((a[:, 2] <= plane_z) & (b[:, 2] >= plane_z))
            | ((b[:, 2] <= plane_z) & (a[:, 2] >= plane_z))
        )
        if not np.any(crossing):
            continue
        t = (plane_z - a[crossing, 2]) / dz[crossing]
        points.append(a[crossing] + t[:, None] * (b[crossing] - a[crossing]))
    if not points:
        return np.empty((0, 3))
    return np.vstack(points)


def rear_deviation(angles: np.ndarray) -> np.ndarray:
    return (angles - math.pi + math.pi) % (2 * math.pi) - math.pi


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", type=Path)
    parser.add_argument("--margin-deg", type=float, default=5.0)
    args = parser.parse_args()

    root = ET.parse(args.urdf).getroot()
    links = {link.get("name"): link for link in root.findall("link")}
    world = load_world(root)
    laser = world["laser_frame"][:3, 3]
    all_deviations: list[np.ndarray] = []

    print("laser_frame_m", np.round(laser, 6).tolist())
    for name, link in links.items():
        if not name.startswith("so101_"):
            continue
        link_points: list[np.ndarray] = []
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            origin = visual.find("origin")
            local = transform(
                origin.get("xyz", "0 0 0") if origin is not None else "0 0 0",
                origin.get("rpy", "0 0 0") if origin is not None else "0 0 0",
            )
            triangles = triangles_from_stl(mesh_path(args.urdf, mesh.get("filename")))
            scale = np.array(list(map(float, mesh.get("scale", "1 1 1").split())))
            triangles *= scale
            matrix = world[name] @ local
            triangles = apply(matrix, triangles.reshape(-1, 3)).reshape(-1, 3, 3)
            points = triangle_plane_points(triangles, laser[2])
            if len(points):
                link_points.append(points)
        if not link_points:
            continue
        points = np.vstack(link_points)
        angles = np.arctan2(points[:, 1] - laser[1], points[:, 0] - laser[0])
        deviations = rear_deviation(angles)
        all_deviations.append(deviations)
        print(
            name,
            "intersection_points", len(points),
            "rear_deviation_deg",
            [round(math.degrees(deviations.min()), 2), round(math.degrees(deviations.max()), 2)],
        )

    if not all_deviations:
        raise SystemExit("no SO-101 mesh intersects the scan plane")
    deviations = np.concatenate(all_deviations)
    geometry_half_deg = max(abs(math.degrees(deviations.min())), abs(math.degrees(deviations.max())))
    recommended_half_deg = math.ceil(geometry_half_deg + args.margin_deg)
    half = math.radians(recommended_half_deg)
    print("geometry_half_width_deg", round(geometry_half_deg, 2))
    print("safety_margin_deg", args.margin_deg)
    print("recommended_half_width_deg", recommended_half_deg)
    print("recommended_half_width_rad", round(half, 6))
    print(
        "mask_0_to_2pi_rad",
        [round(math.pi - half, 6), round(math.pi + half, 6)],
    )
    print(
        "mask_negpi_to_pi_rad",
        [round(-math.pi, 6), round(-math.pi + half, 6)],
        [round(math.pi - half, 6), round(math.pi, 6)],
    )


if __name__ == "__main__":
    main()
