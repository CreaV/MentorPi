"""
Gemini 2 camera bringup (RGB-D, IMU disabled), split out of base.launch.py
so camera_watchdog can restart just the camera when the orbbec driver wedges
("openUsbDevice failed" retry loop after a service restart — the driver never
recovers on its own once open fails; observed 2026-07-05 and 2026-07-12).

Not started directly by base.launch.py — camera_watchdog spawns this as a
supervised subprocess. Manual run for debugging:

    ros2 launch mentorpi_bringup camera.launch.py [enable_imu:=true]

enable_imu turns on the Gemini 2's built-in IMU (accel + gyro, synced output
on /camera/gyro_accel/sample). Off by default: the SLAM stack fuses the STM32
IMU, and the camera IMU is only needed for visual-inertial experiments
(docs/vision_mapping_roadmap.md, S0). base.launch.py forwards its
camera_imu argument here via camera_watchdog.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    orbbec_dir = get_package_share_directory('orbbec_camera')
    enable_imu = LaunchConfiguration('enable_imu')
    imu_rate = LaunchConfiguration('imu_rate')

    return LaunchDescription([
        DeclareLaunchArgument('enable_imu', default_value='false',
            description='Enable the Gemini 2 built-in IMU (VIO experiments)'),
        DeclareLaunchArgument('imu_rate', default_value='200hz',
            description='Camera accel/gyro rate when enable_imu is true'),

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
                'enable_accel': enable_imu,
                'enable_gyro': enable_imu,
                'enable_sync_output_accel_gyro': enable_imu,
                'accel_rate': imu_rate,
                'gyro_rate': imu_rate,
                'enable_colored_point_cloud': 'false',
            }.items(),
        ),

        # 前向低障"虚拟激光": 2D 雷达扫描面离地 0.143m, 更矮的障碍(平放纸箱/
        # 脚/门槛)物理不可见(2026-07-12 实测撞箱)。取深度图光轴中心 ±40 行
        # (约 ±4°)每列最小深度 → /depth_scan, base_node 的避障守卫将其并入
        # 前向扇区。注意 depthimage_to_laserscan 不读 TF 俯仰: 相机光心离地
        # 约 0.104m, 而 2026-07-24 标定的 pitch=-0.1196 rad(抬头约 6.9°)尚未
        # 用平地点云核实; 若属实, 这条高度带会随距离抬高, 低障覆盖变差。
        # 见 docs/vision_mapping_roadmap.md P0。
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
