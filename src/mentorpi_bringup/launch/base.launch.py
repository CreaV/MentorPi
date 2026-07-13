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
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('mentorpi_bringup')
    description_dir = get_package_share_directory('mentorpi_description')
    robot_xacro = os.path.join(description_dir, 'urdf', 'mentorpi.xacro')

    return LaunchDescription([
        # Fixed geometry comes from xacro. Runtime mode omits base_footprint
        # (EKF owns odom->base_link) and Orbbec-owned camera internal frames.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', robot_xacro, ' runtime_mode:=true']),
                    value_type=str,
                ),
            }],
        ),

        # Zero-position /joint_states for the wheel joints (no encoder
        # feedback on this base); without it robot_state_publisher leaves
        # the four continuous wheel joints out of TF and viewers show a
        # wheelless robot model. Reads /robot_description topic.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),

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
        # 由 camera_watchdog 托管 (spawns camera.launch.py): orbbec driver 的
        # USB wedge ("openUsbDevice failed" 后死循环, 服务重启时高发) 需要
        # usbreset + driver 重启才能恢复, watchdog 检测到帧停发时自动做。
        Node(
            package='mentorpi_bringup',
            executable='camera_watchdog',
            name='camera_watchdog',
            output='screen',
        ),

        # Bounded motion primitives (voice / VLA / agent execution substrate).
        # Idle unless an action goal arrives; always stops the base on
        # completion/cancel/timeout/stale-odom.
        Node(
            package='mentorpi_motion',
            executable='motion_node',
            name='mentorpi_motion',
            output='screen',
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
