#!/usr/bin/env python3
"""
export_gs_dataset.py — rtabmap.db → nerfstudio dataset for Gaussian Splatting.

Wraps `rtabmap-export` (ships with rtabmap / ros-jazzy-rtabmap) to dump the
loop-closure-optimized camera poses + RGB(+depth) keyframes out of an RTAB-Map
database, then converts them into a nerfstudio-style dataset:

    <out>/
      transforms.json      camera poses (OpenGL c2w) + per-frame intrinsics
      images/<id>.jpg      RGB keyframes (undistorted if OpenCV is available)
      depth/<id>.png       16UC1 depth in mm (registered to color)
      sparse_pc.ply        assembled RGB-D cloud — seed points for splatfacto

IMPORTANT: poses are kept in the ROS `map` frame (meters, z-up). Train with
auto-orientation/scaling DISABLED so the resulting splat stays aligned with
the SLAM map — then a live robot pose from rtabmap localization drops straight
into the splat with no extra registration:

    ns-train splatfacto --data <out> \
        nerfstudio-data --orientation-method none --center-method none \
        --auto-scale-poses False

See docs/gaussian_splatting.md for the full pipeline.

Runs on any machine with rtabmap tools installed (no ROS runtime needed):
    sudo apt install ros-jazzy-rtabmap   # or build rtabmap standalone
    python3 export_gs_dataset.py ~/rtabmap_maps/rtabmap.db --output-dir ~/gs_dataset
"""

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None


# ---------- OpenCV-YAML calib parsing (no cv2/yaml dependency) ----------

def _parse_mat(text: str, key: str):
    """Extract `data: [ ... ]` for a named matrix from OpenCV YAML text."""
    m = re.search(key + r":.*?data:\s*\[(.*?)\]", text, re.S)
    if m is None:
        return None
    return np.array([float(x) for x in m.group(1).replace("\n", " ").split(",")])


def load_calib(path: Path):
    text = path.read_text()
    K = _parse_mat(text, "camera_matrix").reshape(3, 3)
    D = _parse_mat(text, "distortion_coefficients")
    m = re.search(r"image_width:\s*(\d+)", text)
    width = int(m.group(1))
    height = int(re.search(r"image_height:\s*(\d+)", text).group(1))
    model = re.search(r"distortion_model:\s*\"?(\w+)\"?", text)
    model = model.group(1) if model else "plumb_bob"
    return K, D, width, height, model


# ---------- pose conversion ----------

def quat_to_rot(qx, qy, qz, qw):
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


# ROS optical frame == OpenCV camera (x right, y down, z forward).
# nerfstudio wants OpenGL (x right, y up, z back): flip the y/z basis columns.
CV_TO_GL = np.diag([1.0, -1.0, -1.0])


def camera_pose_to_transform(tx, ty, tz, qx, qy, qz, qw):
    c2w = np.eye(4)
    c2w[:3, :3] = quat_to_rot(qx, qy, qz, qw) @ CV_TO_GL
    c2w[:3, 3] = [tx, ty, tz]
    return c2w


# ---------- main ----------

def run_rtabmap_export(db: Path, work: Path, with_cloud: bool, max_range: float,
                       voxel: float) -> None:
    cmd = [
        "rtabmap-export",
        "--images_id",
        "--poses_camera",
        "--poses_format", "11",   # stamp x y z qx qy qz qw id (ROS frame)
        "--output_dir", str(work),
        "--output", "export",
    ]
    if with_cloud:
        cmd += ["--cloud", "--voxel", str(voxel), "--max_range", str(max_range)]
    cmd.append(str(db))
    print("+", " ".join(cmd))
    subprocess.run(cmd, check=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("database", help="rtabmap .db file")
    ap.add_argument("--output-dir", required=True, help="dataset output directory")
    ap.add_argument("--no-undistort", action="store_true",
                    help="keep original images, write zero distortion (use if cv2 missing)")
    ap.add_argument("--no-cloud", action="store_true", help="skip sparse_pc.ply export")
    ap.add_argument("--max-range", type=float, default=4.0, help="cloud max depth range (m)")
    ap.add_argument("--voxel", type=float, default=0.02, help="cloud voxel filter (m)")
    args = ap.parse_args()

    db = Path(args.database).expanduser().resolve()
    if not db.is_file():
        print(f"database not found: {db}", file=sys.stderr)
        return 2
    if shutil.which("rtabmap-export") is None:
        print("rtabmap-export not in PATH (install ros-jazzy-rtabmap or rtabmap standalone)",
              file=sys.stderr)
        return 2

    out = Path(args.output_dir).expanduser().resolve()
    work = out / "_rtabmap_export"
    (out / "images").mkdir(parents=True, exist_ok=True)
    (out / "depth").mkdir(exist_ok=True)
    work.mkdir(exist_ok=True)

    run_rtabmap_export(db, work, not args.no_cloud, args.max_range, args.voxel)

    poses_file = work / "export_camera_poses.txt"
    rgb_dir = work / "export_rgb"
    depth_dir = work / "export_depth"
    calib_dir = work / "export_calib"

    undistort = (not args.no_undistort) and cv2 is not None
    if not args.no_undistort and cv2 is None:
        print("[warn] cv2 not available — skipping undistortion, writing raw distortion "
              "coefficients (nerfstudio only supports 4-term OPENCV; rational_polynomial "
              "will be zeroed)", file=sys.stderr)

    frames = []
    skipped = 0
    for line in poses_file.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        vals = line.split()
        stamp = float(vals[0])
        tx, ty, tz, qx, qy, qz, qw = map(float, vals[1:8])
        node_id = vals[8] if len(vals) > 8 else None
        if node_id is None:
            print("[error] poses file has no node ids — need --poses_format 11",
                  file=sys.stderr)
            return 1

        rgb = rgb_dir / f"{node_id}.jpg"
        if not rgb.is_file():
            rgb = rgb_dir / f"{node_id}.png"
        calib = calib_dir / f"{node_id}.yaml"
        if not rgb.is_file() or not calib.is_file():
            skipped += 1
            continue

        K, D, width, height, model = load_calib(calib)
        img_rel = f"images/{node_id}{rgb.suffix}"

        if undistort and D is not None and np.any(D != 0.0):
            img = cv2.imread(str(rgb), cv2.IMREAD_UNCHANGED)
            img = cv2.undistort(img, K, D)  # keeps K as the new projection
            cv2.imwrite(str(out / img_rel), img)
            dist = np.zeros(4)
        else:
            shutil.copyfile(rgb, out / img_rel)
            # nerfstudio OPENCV model only takes k1 k2 p1 p2
            dist = (D[:4] if D is not None and model == "plumb_bob" else np.zeros(4))

        frame = {
            "file_path": img_rel,
            "fl_x": K[0, 0], "fl_y": K[1, 1],
            "cx": K[0, 2], "cy": K[1, 2],
            "w": width, "h": height,
            "k1": dist[0], "k2": dist[1], "p1": dist[2], "p2": dist[3],
            "camera_model": "OPENCV",
            "transform_matrix": camera_pose_to_transform(tx, ty, tz, qx, qy, qz, qw).tolist(),
            "timestamp": stamp,
        }

        depth = depth_dir / f"{node_id}.png"
        if depth.is_file():
            depth_rel = f"depth/{node_id}.png"
            shutil.copyfile(depth, out / depth_rel)
            frame["depth_file_path"] = depth_rel

        frames.append(frame)

    if not frames:
        print("[error] no usable frames exported", file=sys.stderr)
        return 1

    transforms = {
        "camera_model": "OPENCV",
        # depth png is uint16 millimeters
        "depth_unit_scale_factor": 0.001,
        "frames": sorted(frames, key=lambda f: f["timestamp"]),
    }

    cloud = work / "export_cloud.ply"
    if cloud.is_file():
        shutil.copyfile(cloud, out / "sparse_pc.ply")
        transforms["ply_file_path"] = "sparse_pc.ply"

    (out / "transforms.json").write_text(json.dumps(transforms, indent=2))

    print(f"\ndataset written to {out}")
    print(f"  frames: {len(frames)} (skipped {skipped} without image/calib)")
    print(f"  seed cloud: {'yes' if cloud.is_file() else 'no'}")
    print("\nnext (on the training server):")
    print(f"  ns-train splatfacto --data {out} \\")
    print("      nerfstudio-data --orientation-method none --center-method none \\")
    print("      --auto-scale-poses False")
    print("  ns-export gaussian-splat --load-config outputs/.../config.yml "
          "--output-dir exports/splat")
    print("\nthe exported splat.ply stays in the SLAM map frame — "
          "view it with scripts/live_rerun.py --splat splat.ply")
    return 0


if __name__ == "__main__":
    sys.exit(main())
