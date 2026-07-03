"""
3D SLAM mode-only launch. Adds rtabmap + colored point cloud on top of the
already-running base (which provides EKF odom and the Gemini 2L RGB-D
streams). Started/stopped by mentorpi_supervisor.

Heterogeneous architecture: rtabmap reads odom from TF (odom->base_link by
EKF), not from rgbd_odometry. See CLAUDE.md.

The MS200 lidar scan is fused in as well: with no wheel encoders the EKF odom
is open-loop and slips; RGBD/NeighborLinkRefining scan-matches consecutive
nodes to correct it, and RGBD/ProximityBySpace adds lidar proximity links when
revisiting places (works even where visual loop closure fails, e.g. blank
walls / dim light).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Shared between slam_3d and loc_3d (keep in sync with loc_3d.launch.py).
RTABMAP_TOPIC_PARAMS = {
    'frame_id': 'base_link',
    'odom_frame_id': 'odom',
    'subscribe_depth': True,
    'subscribe_rgb': True,
    'subscribe_scan': True,
    'subscribe_odom_info': False,
    'approx_sync': True,
    'topic_queue_size': 20,
    'sync_queue_size': 20,
    # MS200 driver publishes /scan with SensorDataQoS (best effort);
    # a reliable subscription would never match it.
    'qos_scan': 2,
}

RTABMAP_TUNING_PARAMS = {
    # Ground robot: lock roll/pitch/z out of the pose graph.
    'Reg/Force3DoF': 'true',
    # Visual + ICP registration: loop closures found visually get refined
    # with the lidar scan instead of trusting the (slipping) odom guess.
    'Reg/Strategy': '1',
    # Scan-match consecutive nodes against odom — the main wheel-slip fix.
    'RGBD/NeighborLinkRefining': 'true',
    # Lidar proximity detection when driving back through a mapped area.
    'RGBD/ProximityBySpace': 'true',
    'RGBD/ProximityPathMaxNeighbors': '10',
    # ICP settings for a sparse 450-point 2D scan.
    'Icp/VoxelSize': '0.05',
    'Icp/MaxCorrespondenceDistance': '0.15',
    'Icp/CorrespondenceRatio': '0.2',
    'Icp/MaxTranslation': '0.5',
    'Icp/PointToPlane': 'false',
    # Pi 5 budget.
    'Rtabmap/DetectionRate': '2.0',
    'RGBD/OptimizeMaxError': '3.0',
    'Kp/MaxFeatures': '300',
    # Occupancy grid from lidar + depth (0=scan, 1=depth, 2=both).
    'Grid/Sensor': '2',
    'Grid/MaxGroundHeight': '0.05',
    'Grid/MaxObstacleHeight': '1.5',
    'Grid/RangeMax': '5.0',
    'Grid/3D': 'true',
    'GridGlobal/MinSize': '20.0',
}

RTABMAP_REMAPPINGS = [
    ('rgb/image', '/camera/color/image_raw'),
    ('rgb/camera_info', '/camera/color/camera_info'),
    ('depth/image', '/camera/depth/image_raw'),
    ('scan', '/scan'),
    ('odom', '/odometry/filtered'),
]


def generate_launch_description():
    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value='~/rtabmap_maps/rtabmap.db',
        description='Path to RTAB-Map database file')
    database_path = LaunchConfiguration('database_path')

    return LaunchDescription([
        database_path_arg,

        # rtabmap (mapping + loop closure)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                **RTABMAP_TOPIC_PARAMS,
                **RTABMAP_TUNING_PARAMS,
                'database_path': database_path,
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
            }],
            remappings=RTABMAP_REMAPPINGS,
        ),

        # Decimated colored point cloud for RViz / Foxglove visualization
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
