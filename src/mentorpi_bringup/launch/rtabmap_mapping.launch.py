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

    # Shared RTAB-Map parameters
    rtabmap_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'approx_sync': True,
        'queue_size': 10,
    }

    return LaunchDescription([
        localization_arg,
        database_path_arg,

        # --- Hardware ---

        # 1. 串口驱动 (Base) — 轮式里程计发布 /odom，不发TF
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

        # 2. Gemini 2L 深度相机 (RGB-D + IMU)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(orbbec_dir, 'launch', 'gemini2L.launch.py')
            ),
            launch_arguments={
                'color_width': '640',
                'color_height': '480',
                'color_fps': '30',
                'color_format': 'MJPG',
                'depth_width': '640',
                'depth_height': '400',
                'depth_fps': '30',
                'depth_registration': 'true',
                'enable_accel': 'true',
                'enable_gyro': 'true',
                'enable_sync_output_accel_gyro': 'true',
                'enable_colored_point_cloud': 'true',
            }.items(),
        ),

        # 2b. IMU 滤波 (Madgwick) — 给 Gemini IMU 补上 orientation
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='camera_imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
                'gain': 0.1,
                'frequency': 100.0,
            }],
            remappings=[
                ('/imu/data_raw', '/camera/gyro_accel/sample'),
                ('/imu/data', '/camera/imu/data'),
            ],
        ),

        # 3. 手柄 + 遥控
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='mentorpi_teleop',
            executable='teleop_node',
            name='mentorpi_teleop',
            output='screen'
        ),

        # 4. TF: base_link → camera_link (相机安装位置，根据实际调整)
        #    x=前方0.05m, y=0, z=高0.15m, 无旋转
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=['0.05', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link'],
        ),

        # 5. TF: base_link → laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser_frame'],
        ),
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

        # --- RTAB-Map ---

        # 5. Visual Odometry (RGB-D odometry)
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                **rtabmap_params,
                'publish_tf': True,          # odom → base_link
                'wait_imu_to_init': True,     # 等 IMU 初始化后再开始
                'Odom/Strategy': '0',         # 0=Frame-to-Map (more robust)
                'Vis/MinInliers': '10',       # 默认20，降低以减少匹配失败
                'Vis/MaxFeatures': '700',     # 多提特征，增加匹配成功率
                'Vis/InlierDistance': '0.1',  # 放宽 inlier 距离阈值
                'Odom/ResetCountdown': '5',   # 连续失败5帧才 reset
                'Odom/GuessSmoothingDelay': '0.5',
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('imu', '/camera/imu/data'),
            ],
        ),

        # 6. RTAB-Map SLAM
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                **rtabmap_params,
                'database_path': database_path,
                'Mem/IncrementalMemory': 'true',   # mapping mode
                'Mem/InitWMWithAllNodes': 'false',
                # 3D map
                'Grid/FromDepth': 'true',
                'Grid/MaxGroundHeight': '0.05',
                'Grid/MaxObstacleHeight': '1.5',
                'Grid/RangeMax': '5.0',
                'Grid/3D': 'true',
                'GridGlobal/MinSize': '20.0',
                # Loop closure
                'Rtabmap/DetectionRate': '2.0',       # Hz, Pi 5 friendly
                'RGBD/OptimizeMaxError': '3.0',
                'Kp/MaxFeatures': '300',
                # Visualization
                'Rtabmap/CreateIntermediateNodes': 'true',
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
            ],
        ),

        # 7. 点云发布 (用于 RViz2 实时可视化)
        Node(
            package='rtabmap_util',
            executable='point_cloud_xyzrgb',
            name='point_cloud_xyzrgb',
            output='screen',
            parameters=[{
                'approx_sync': True,
                'decimation': 4,        # 降采样，减轻 Pi 5 负担
                'voxel_size': 0.05,
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
