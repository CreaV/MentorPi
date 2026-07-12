"""
Gemini 2L camera bringup (RGB-D, IMU disabled), split out of base.launch.py
so camera_watchdog can restart just the camera when the orbbec driver wedges
("openUsbDevice failed" retry loop after a service restart — the driver never
recovers on its own once open fails; observed 2026-07-05 and 2026-07-12).

Not started directly by base.launch.py — camera_watchdog spawns this as a
supervised subprocess. Manual run for debugging:

    ros2 launch mentorpi_bringup camera.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    orbbec_dir = get_package_share_directory('orbbec_camera')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(orbbec_dir, 'launch', 'gemini2L.launch.py')
            ),
            launch_arguments={
                'color_width': '640',
                'color_height': '480',
                'color_fps': '15',
                'color_format': 'MJPG',
                'depth_width': '640',
                'depth_height': '400',
                'depth_fps': '15',
                'depth_registration': 'true',
                'enable_accel': 'false',
                'enable_gyro': 'false',
                'enable_sync_output_accel_gyro': 'false',
                'enable_colored_point_cloud': 'false',
            }.items(),
        ),
    ])
