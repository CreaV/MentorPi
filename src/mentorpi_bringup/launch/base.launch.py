"""
Always-on base hardware. Started by remote.launch.py and remains running
across mode switches handled by mentorpi_supervisor.

Includes: base_node (serial), STM32 IMU Madgwick filter, EKF, Gemini 2L
RGB-D camera (always-on preview source), joystick + teleop, lidar, and
the base_link static TF tree.

The Gemini 2L lives here (not in slam_3d) so /camera/color/image_raw is
available across all modes for live preview. Switching to slam_3d adds
rtabmap on top of the already-running camera streams.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('mentorpi_bringup')
    orbbec_dir = get_package_share_directory('orbbec_camera')

    return LaunchDescription([
        # Serial driver -- /odom 50Hz + /imu/data_raw. odom TF provided by EKF.
        Node(
            package='mentorpi_base',
            executable='base_node',
            name='mentorpi_base',
            parameters=[{
                # Stable USB by-id path: ttyACMx number changes after USB
                # re-enumeration (observed ttyACM0 -> ttyACM1 mid-session),
                # which left base_node retrying a dead path and silently broke
                # cmd_vel/gimbal/buzzer until reboot.
                'port': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B21250490-if00',
                'baudrate': 1000000,
                'publish_odom_tf': False,
            }],
            output='screen',
        ),

        # STM32 IMU Madgwick filter -- /imu/data_raw -> /imu/data
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            output='screen',
            parameters=[os.path.join(bringup_dir, 'config', 'imu_filter.yaml')],
        ),

        # EKF -- /odom + /imu/data -> /odometry/filtered + TF odom->base_link
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(bringup_dir, 'config', 'ekf.yaml')],
        ),

        # Gemini 2L (RGB-D, IMU disabled). Always on so preview / 3D SLAM share
        # the same camera instance -- starting slam_3d does NOT relaunch camera.
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

        # Joystick (joy_linux bypasses SDL2 issues with the BTP-KP20D dongle).
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_node',
            output='screen',
        ),

        # Teleop -- /joy -> /cmd_vel + /gimbal/cmd
        Node(
            package='mentorpi_teleop',
            executable='teleop_node',
            name='mentorpi_teleop',
            output='screen',
        ),

        # base_link -> laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser_frame'],
        ),

        # base_link -> imu_link (STM32 IMU on the controller board). Without
        # this TF robot_localization cannot transform /imu/data (frame_id
        # 'imu_link') into base_link and silently drops every IMU measurement.
        # Translation barely matters for gyro/orientation fusion; axis
        # alignment does — adjust RPY here if the board is mounted rotated.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link'],
        ),

        # base_link -> camera_link (placeholder extrinsic; needs calibration).
        # Lives in base because the camera is now always running.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=['0.05', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link'],
        ),

        # MS200 lidar
        Node(
            package='oradar_lidar',
            executable='oradar_scan',
            name='oradar_ros',
            output='screen',
            parameters=[
                {'device_model': 'MS200'},
                {'frame_id': 'laser_frame'},
                {'scan_topic': '/scan'},
                {'port_name': '/dev/ttyUSB0'},
                {'baudrate': 230400},
                {'angle_min': 0.0},
                {'angle_max': 360.0},
                {'range_min': 0.05},
                {'range_max': 12.0},
                {'clockwise': False},
                {'motor_speed': 10},
            ],
        ),
    ])
