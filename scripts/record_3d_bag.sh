#!/usr/bin/env bash
# Record a rosbag of all topics needed for offline 3D-SLAM debugging.
#
#   ./record_3d_bag.sh [run_name]
#
# Defaults output dir to ~/rosbags/3d_slam/<timestamp>/. Format is MCAP.
# Requires:  sudo apt install ros-jazzy-rosbag2-storage-mcap
#
# Run rtabmap_mapping.launch.py in a separate terminal first (or just
# mentorpi.launch.py if you only want sensor data without rtabmap).

set -euo pipefail

if [ -z "${ROS_DISTRO:-}" ]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
fi

OUT_BASE="${OUT_BASE:-$HOME/rosbags/3d_slam}"
NAME="${1:-$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="$OUT_BASE/$NAME"

if [ -e "$OUT_DIR" ]; then
    echo "Output already exists: $OUT_DIR" >&2
    exit 1
fi
mkdir -p "$OUT_BASE"

TOPICS=(
    # TF tree
    /tf
    /tf_static
    # Odometry: raw wheel + EKF-fused
    /odom
    /odometry/filtered
    # IMU: raw + Madgwick-filtered (with orientation)
    /imu/data_raw
    /imu/data
    # Gemini 2L RGB-D + intrinsics (depth is HW-registered to color)
    /camera/color/image_raw
    /camera/color/camera_info
    /camera/depth/image_raw
    /camera/depth/camera_info
    # MS200 lidar
    /scan
    # Velocity command (useful for replay correlation)
    /cmd_vel
    # Optional rtabmap outputs — uncomment to capture map state during run.
    # Each /rtabmap/cloud_map message can be tens of MB; only enable for short runs.
    # /rtabmap/odom
    # /rtabmap/grid_map
    # /rtabmap/cloud_map
    # /rtabmap/info
)

echo "Recording 3D-SLAM bag → $OUT_DIR"
echo "Topics:"
printf '  %s\n' "${TOPICS[@]}"
echo "Press Ctrl-C to stop."

exec ros2 bag record \
    --storage mcap \
    --output "$OUT_DIR" \
    "${TOPICS[@]}"
