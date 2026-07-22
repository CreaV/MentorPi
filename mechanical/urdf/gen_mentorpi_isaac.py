"""Isaac-flavored MentorPi base URDF (NO arm) for Isaac Sim / Isaac Lab import.

Arm-less variant: mecanum base + camera + lidar + imu (`with_so101:=false`).
For the full mobile manipulator see `gen_mentorpi_so101_isaac.py`. Everything
else (calibration binding, absolute mesh paths, floating-base import notes)
is identical — see that file's docstring.

BOUND TO CALIBRATION: generated from `mecanum.xacro`. Re-run
`isaac/export_isaac.sh` after any calibration/xacro change to re-sync.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
XACRO = REPO_ROOT / "src/mentorpi_description/urdf/mentorpi.xacro"
MESH_ROOT = REPO_ROOT / "src/mentorpi_description"
OUT = REPO_ROOT / "isaac/mentorpi.isaac.urdf"
WITH_SO101 = "false"


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
    return result.stdout.replace(
        "package://mentorpi_description/", f"{MESH_ROOT}/"
    )


if __name__ == "__main__":
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(gen_urdf())
    print(f"wrote {OUT}")
