import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('mentorpi_bringup')
    camera_type = LaunchConfiguration('camera_type')

    return LaunchDescription([
        # 参数: camera_type = "opencv" (默认) 或 "gemini2l"
        DeclareLaunchArgument('camera_type', default_value='opencv',
            description='Camera: opencv or gemini2l'),

        # 1. 串口驱动 (Base) — odom TF由EKF发布
        Node(
            package='mentorpi_base',
            executable='base_node',
            name='mentorpi_base',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baudrate': 1000000,
                'publish_odom_tf': False,
            }],
            output='screen'
        ),

        # 1b. IMU 滤波 (Madgwick) — /imu/data_raw → /imu/data
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            output='screen',
            parameters=[
                os.path.join(bringup_dir, 'config', 'imu_filter.yaml')
            ],
            remappings=[
                ('/imu/data_raw', '/imu/data_raw'),
                ('/imu/data', '/imu/data'),
            ],
        ),

        # 1c. EKF (robot_localization) — /odom + /imu/data → /odometry/filtered + TF odom→base_link
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(bringup_dir, 'config', 'ekf.yaml')
            ],
        ),

        # 2a. OpenCV 相机 (camera_type == opencv)
        Node(
            package='mentorpi_vision',
            executable='vision_node',
            name='mentorpi_vision',
            output='screen',
            condition=IfCondition(PythonExpression(["'", camera_type, "' == 'opencv'"])),
        ),

        # 2b. Orbbec Gemini 2L 深度相机 (camera_type == gemini2l)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('orbbec_camera'),
                    'launch', 'gemini2L.launch.py'
                )
            ),
            launch_arguments={
                'color_width': '640',
                'color_height': '480',
                'color_fps': '30',
                'color_format': 'MJPG',
                'depth_width': '640',
                'depth_height': '400',
                'depth_fps': '30',
            }.items(),
            condition=IfCondition(PythonExpression(["'", camera_type, "' == 'gemini2l'"])),
        ),

        # 3. 手柄驱动 (ROS 2 Joy)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # 4. 远程控制逻辑 (Teleop)
        Node(
            package='mentorpi_teleop',
            executable='teleop_node',
            name='mentorpi_teleop',
            output='screen'
        ),

        # 5. TF: base_link → laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser_frame'],
        ),

        # 6. 激光雷达 (MS200)
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
