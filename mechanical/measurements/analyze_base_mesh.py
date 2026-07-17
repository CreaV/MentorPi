#!/usr/bin/env python3
"""Extract mounting candidates from the existing binary base_link STL."""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

import numpy as np
from sklearn.cluster import DBSCAN


def read_binary_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    normals = np.empty((count, 3), dtype=float)
    triangles = np.empty((count, 3, 3), dtype=float)
    for index in range(count):
        offset = 84 + index * 50
        normals[index] = struct.unpack_from("<3f", data, offset)
        for vertex in range(3):
            triangles[index, vertex] = struct.unpack_from(
                "<3f", data, offset + 12 + vertex * 12
            )
    return normals, triangles


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("stl", type=Path)
    args = parser.parse_args()
    normals, triangles = read_binary_stl(args.stl)
    points = triangles.reshape(-1, 3)
    print("bounds_m", points.min(axis=0).round(6).tolist(), points.max(axis=0).round(6).tolist())

    z_span = np.ptp(triangles[:, :, 2], axis=1)
    horizontal_normals = np.abs(normals[:, 2]) < 0.15
    vertical_faces = triangles[horizontal_normals & (z_span > 0.001)]
    centroids = vertical_faces.mean(axis=1)
    labels = DBSCAN(eps=0.0035, min_samples=3).fit_predict(centroids[:, :2])
    candidates = []
    for label in sorted(set(labels) - {-1}):
        cluster_faces = vertical_faces[labels == label]
        cluster_points = cluster_faces.reshape(-1, 3)
        lo = cluster_points.min(axis=0)
        hi = cluster_points.max(axis=0)
        size = hi - lo
        if size[0] <= 0.018 and size[1] <= 0.018 and hi[2] >= 0.025:
            candidates.append((cluster_points[:, :2].mean(axis=0), lo, hi, len(cluster_faces)))
    for center, lo, hi, face_count in sorted(candidates, key=lambda item: (item[0][0], item[0][1])):
        print(
            "candidate",
            "center_mm=", np.round(center * 1000, 2).tolist(),
            "xy_size_mm=", np.round((hi - lo)[:2] * 1000, 2).tolist(),
            "z_mm=", np.round([lo[2], hi[2]] * np.array([1000, 1000]), 2).tolist(),
            "faces=", face_count,
        )


if __name__ == "__main__":
    main()
