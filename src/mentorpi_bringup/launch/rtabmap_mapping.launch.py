import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('mentorpi_bringup')
    orbbec_dir = get_package_share_directory('orbbec_camera')

    localization_arg = DeclareLaunchArgument(
        'localization', default_value='false',
        description='Set to true to run in localization mode (use existing map)')

    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value='~/rtabmap_maps/rtabmap.db',
        description='Path to RTAB-Map database file')

    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')

    # 异构架构：odom 由底盘 EKF 提供（高频低延迟），
    # rtabmap 仅做 mapping + loop closure（低频）
    rtabmap_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_odom_info': False,    # no rgbd_odometry
        'approx_sync': True,
        'topic_queue_size': 20,
        'sync_queue_size': 20,
    }

    return LaunchDescription([
        localization_arg,
        database_path_arg,

        # ───── 底盘 + 里程计前端（高频 dead-reckoning） ─────

        # 1. 串口驱动 (Base) — /odom 50Hz + /imu/data_raw
        Node(
            package='mentorpi_base',
            executable='base_node',
            name='mentorpi_base',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baudrate': 1000000,
                'publish_odom_tf': False,   # TF 由 EKF 发
            }],
            output='screen'
        ),

        # 1b. STM32 IMU 滤波 (Madgwick) — /imu/data_raw → /imu/data
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            parameters=[
                os.path.join(bringup_dir, 'config', 'imu_filter.yaml')
            ],
            output='screen',
        ),

        # 1c. EKF — /odom + /imu/data → /odometry/filtered + TF odom→base_link
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[
                os.path.join(bringup_dir, 'config', 'ekf.yaml')
            ],
            output='screen',
        ),

        # ───── 相机（仅 RGB-D，IMU 不需要） ─────

        # 2. Gemini 2L
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

        # ───── 遥控 ─────

        Node(package='joy', executable='joy_node', name='joy_node', output='screen'),
        Node(package='mentorpi_teleop', executable='teleop_node',
             name='mentorpi_teleop', output='screen'),

        # ───── 静态 TF ─────

        # base_link → camera_link (前 0.05m, 高 0.15m)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=['0.05', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link'],
        ),
        # base_link → laser_frame (高 0.18m)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser_frame'],
        ),

        # ───── 激光雷达（仅避障/可视化，不参与 3D 建图） ─────

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

        # ───── RTAB-Map 后端（mapping + loop closure，1-2Hz） ─────

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                **rtabmap_params,
                'database_path': database_path,
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                # 3D map
                'Grid/FromDepth': 'true',
                'Grid/MaxGroundHeight': '0.05',
                'Grid/MaxObstacleHeight': '1.5',
                'Grid/RangeMax': '5.0',
                'Grid/3D': 'true',
                'GridGlobal/MinSize': '20.0',
                # Loop closure
                'Rtabmap/DetectionRate': '2.0',
                'RGBD/OptimizeMaxError': '3.0',
                'Kp/MaxFeatures': '300',
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('odom', '/odometry/filtered'),
            ],
        ),

        # 点云发布 (RViz2 可视化)
        Node(
            package='rtabmap_util',
            executable='point_cloud_xyzrgb',
            name='point_cloud_xyzrgb',
            output='screen',
            parameters=[{
                'approx_sync': True,
                'decimation': 8,
                'voxel_size': 0.10,
                'max_depth': 5.0,
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('cloud', '/rtabmap/cloud'),
            ],
        ),
    ])
