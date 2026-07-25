"""Generate the standalone MentorPi + SO-101 URDF from project xacro sources."""

from __future__ import annotations

import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
XACRO = REPO_ROOT / "src/mentorpi_description/urdf/mentorpi.xacro"


def gen_urdf() -> str:
    result = subprocess.run(
        [
            "/opt/ros/jazzy/bin/xacro",
            str(XACRO),
            "runtime_mode:=false",
            "with_so101:=true",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout

