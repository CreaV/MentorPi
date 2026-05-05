"""
3D SLAM mode-only launch. Adds rtabmap + colored point cloud on top of the
already-running base (which provides EKF odom and the Gemini 2L RGB-D
streams). Started/stopped by mentorpi_supervisor.

Heterogeneous architecture: rtabmap reads odom from TF (odom->base_link by
EKF), not from rgbd_odometry. See CLAUDE.md.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value='~/rtabmap_maps/rtabmap.db',
        description='Path to RTAB-Map database file')
    database_path = LaunchConfiguration('database_path')

    rtabmap_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'topic_queue_size': 20,
        'sync_queue_size': 20,
    }

    return LaunchDescription([
        database_path_arg,

        # rtabmap (mapping + loop closure)
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
                'Grid/FromDepth': 'true',
                'Grid/MaxGroundHeight': '0.05',
                'Grid/MaxObstacleHeight': '1.5',
                'Grid/RangeMax': '5.0',
                'Grid/3D': 'true',
                'GridGlobal/MinSize': '20.0',
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
