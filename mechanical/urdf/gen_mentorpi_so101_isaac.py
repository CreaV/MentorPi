"""Isaac-flavored MentorPi + SO-101 URDF for Isaac Sim / Isaac Lab import.

Full mobile manipulator: mecanum base + camera + lidar + SO-101 arm
(`with_so101:=true`). For the arm-less base variant see
`gen_mentorpi_isaac.py`.

BOUND TO CALIBRATION: the output is generated from `mecanum.xacro`, the same
source of truth that holds the calibrated wheelbase/track_width/wheel_diameter,
the `camera_joint` extrinsic, and the `laser_joint`. Re-run the one-command
exporter `isaac/export_isaac.sh` after any calibration or xacro change (e.g.
after `calibrate_camera_extrinsic.py --update-xacro`) to re-sync the Isaac
files. Never hand-edit the generated `.isaac.urdf`.

Differs from the ROS URDF only in mesh references: Isaac's URDF importer does
not resolve `package://` URIs, so this rewrites `package://mentorpi_description/`
to an ABSOLUTE path under the repo (foolproof single-machine import; REPO_ROOT
is resolved at generation time, so regeneration adapts if the repo moves).

Collisions are KEPT (Isaac needs them for physics; it convexifies meshes on
import). Joint drive gains are NOT baked here — set them in the Isaac Lab
ArticulationCfg (`isaac/mentorpi_articulation_cfg.py`). Import as a FLOATING
base (fix base link OFF) with merge-fixed-joints ON so the sensor frames
collapse into base_link while the 4 wheels and 6 arm joints stay articulated.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
XACRO = REPO_ROOT / "src/mentorpi_description/urdf/mentorpi.xacro"
MESH_ROOT = REPO_ROOT / "src/mentorpi_description"
OUT = REPO_ROOT / "isaac/mentorpi_so101.isaac.urdf"
WITH_SO101 = "true"


def gen_urdf() -> str:
    result = subprocess.run(
        [
            "/opt/ros/jazzy/bin/xacro",
            str(XACRO),
            "runtime_mode:=false",
            f"with_so101:={WITH_SO101}",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    # package://mentorpi_description/... -> /abs/repo/src/mentorpi_description/...
    return result.stdout.replace(
        "package://mentorpi_description/", f"{MESH_ROOT}/"
    )


if __name__ == "__main__":
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(gen_urdf())
    print(f"wrote {OUT}")
