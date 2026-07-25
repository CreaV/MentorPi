"""Viewer-flavored MentorPi + SO-101 URDF: mesh paths relative to the file.

The text-to-cad CAD Viewer resolves URDF mesh references relative to the
URDF file itself — its frontend does not resolve package:// URIs (they fall
through to the dist static root and 404). This variant rewrites
package://mentorpi_description/ to ../ and exists ONLY for visual review;
ROS consumers keep mentorpi_so101.urdf.

Open with an ABSOLUTE file= query so sibling mesh resolution works:
  http://127.0.0.1:4178/?dir=<repo>&file=<repo>/src/mentorpi_description/urdf/mentorpi_so101.viewer.urdf
"""

from __future__ import annotations

import re
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
    urdf = result.stdout.replace("package://mentorpi_description/", "../")
    # Preview-only speedup: drop collision blocks. Visual+collision reference
    # the same ~60 MB of STLs; skipping collisions halves browser-side
    # parsing. Physics/planning consumers must use mentorpi_so101.urdf.
    return re.sub(r"[ \t]*<collision>.*?</collision>\n?", "", urdf, flags=re.DOTALL)
