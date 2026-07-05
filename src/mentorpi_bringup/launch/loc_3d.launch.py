"""
3D localization mode-only launch. Loads an existing rtabmap database in
localization mode (map is not modified) and republishes map->odom once the
robot relocalizes against it. Assumes base.launch.py is already running.

Use this after a reboot to recover the robot's pose inside a previously
built 3D map — e.g. so live_rerun.py can show the robot + camera frustum
inside the offline Gaussian-splat / point-cloud model of the same map.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Keep these dicts in sync with slam_3d.launch.py (launch files are not
    # importable as modules from each other under ament, so they are inlined).
    topic_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': True,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'topic_queue_size': 20,
        'sync_queue_size': 20,
        'qos_scan': 2,
    }

    tuning_params = {
        'Reg/Force3DoF': 'true',
        'Reg/Strategy': '1',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.15',
        'Icp/CorrespondenceRatio': '0.2',
        'Icp/MaxTranslation': '0.5',
        'Icp/PointToPlane': 'false',
        'Rtabmap/DetectionRate': '2.0',
        # 定位模式只读不写库, 门槛可以比建图松: 薄图(短扫描/少回环)下
        # 3.0 会把仅略超阈值的正确定位也拒掉 (实测 3.11 被拒)。
        'RGBD/OptimizeMaxError': '5.0',
        'Kp/MaxFeatures': '300',
        'Grid/Sensor': '2',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '1.5',
        'Grid/RangeMax': '5.0',
        'Grid/3D': 'true',
        'GridGlobal/MinSize': '20.0',
    }

    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value='~/rtabmap_maps/rtabmap.db',
        description='Existing RTAB-Map database to localize against')
    database_path = LaunchConfiguration('database_path')

    return LaunchDescription([
        database_path_arg,

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                **topic_params,
                **tuning_params,
                'database_path': database_path,
                # Localization mode: read-only map, relocalize + track.
                'Mem/IncrementalMemory': 'false',
                'Mem/InitWMWithAllNodes': 'true',
                # Start from the last saved localization instead of the map
                # origin — usually much closer to the truth after a reboot.
                'RGBD/SavedLocalizationIgnored': 'false',
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('scan', '/scan'),
                ('odom', '/odometry/filtered'),
            ],
        ),

        # Live colored point cloud (preview only; the map itself is frozen).
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
