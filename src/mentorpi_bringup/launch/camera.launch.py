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
from launch_ros.actions import Node
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

        # 前向低障"虚拟激光": 2D 雷达装在 0.18m 高, 更矮的障碍(平放纸箱/
        # 脚/门槛)物理不可见(2026-07-12 实测撞箱)。深度相机装在 0.095m
        # 且无俯仰, 取光轴中心 ±40 行(±~4°, 即距地约 5~14cm 高度带)每列
        # 最小深度 → /depth_scan, base_node 的避障守卫将其并入前向扇区。
        # 向下的行会在 >1.3m 处打到地板, 远超守卫减速区(0.75m), 无害。
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_low_scan',
            output='screen',
            remappings=[
                ('depth', '/camera/depth/image_raw'),
                ('depth_camera_info', '/camera/depth/camera_info'),
                ('scan', '/depth_scan'),
            ],
            parameters=[{
                'scan_height': 80,
                'range_min': 0.2,
                'range_max': 2.0,
                'output_frame': 'camera_depth_frame',
            }],
        ),
    ])
