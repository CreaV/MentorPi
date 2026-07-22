#!/usr/bin/env bash
# One-command Isaac export, bound to the calibrated URDF source of truth.
#
# Regenerates BOTH Isaac URDFs from src/mentorpi_description/urdf/mecanum.xacro
# (which holds the calibrated wheelbase/track_width/wheel_diameter, the
# camera_joint extrinsic, and laser_joint). Run this after ANY calibration or
# xacro change — e.g. after `calibrate_camera_extrinsic.py --update-xacro` — so
# the Isaac files never drift from the real robot.
#
#   bash isaac/export_isaac.sh
#
# Outputs (absolute mesh paths, import-ready):
#   isaac/mentorpi.isaac.urdf         base only (mecanum + camera + lidar)
#   isaac/mentorpi_so101.isaac.urdf   full mobile manipulator (+ SO-101 arm)
# NB: no `-u` (nounset) — ROS setup.bash references unbound vars and would abort.
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

echo "== source ROS + build mentorpi_description (sync install xacro with src) =="
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mentorpi_description >/dev/null
# mentorpi.xacro pulls mecanum.xacro via $(find mentorpi_description) -> install
export AMENT_PREFIX_PATH="$REPO_ROOT/install/mentorpi_description:/opt/ros/jazzy"

echo "== generate Isaac URDFs from the calibrated xacro =="
/usr/bin/python3 mechanical/urdf/gen_mentorpi_isaac.py
/usr/bin/python3 mechanical/urdf/gen_mentorpi_so101_isaac.py

echo "== validate =="
for f in isaac/mentorpi.isaac.urdf isaac/mentorpi_so101.isaac.urdf; do
    check_urdf "$f" | head -1
done

echo "== done. Import into Isaac Sim (floating base, merge fixed joints) — see isaac/README.md =="
