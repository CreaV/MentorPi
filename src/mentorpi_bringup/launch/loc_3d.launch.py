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

from mentorpi_bringup import rtabmap_params as rp


def generate_launch_description():
    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value='~/rtabmap_maps/rtabmap.db',
        description='Existing RTAB-Map database to localize against')
    database_path = LaunchConfiguration('database_path')

    return LaunchDescription([
        database_path_arg,

        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            parameters=[rp.RGBD_SYNC_PARAMS],
            remappings=rp.RGBD_SYNC_REMAPPINGS,
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                # Shared with slam_3d (mentorpi_bringup/rtabmap_params.py),
                # plus the looser localization-only OptimizeMaxError.
                **rp.loc_params(),
                'database_path': database_path,
                # Localization mode: read-only map, relocalize + track.
                'Mem/IncrementalMemory': 'false',
                'Mem/InitWMWithAllNodes': 'true',
                # Start from the last saved localization instead of the map
                # origin — usually much closer to the truth after a reboot.
                'RGBD/SavedLocalizationIgnored': 'false',
            }],
            remappings=rp.REMAPPINGS,
            # 定位模式退出时也会写库 (保存最后定位位姿), 同样给足时间。
            sigterm_timeout=rp.RTABMAP_SIGTERM_TIMEOUT,
            sigkill_timeout=rp.RTABMAP_SIGKILL_TIMEOUT,
        ),

        # 注: 定位模式不再起 point_cloud_xyzrgb —— 实时彩色点云只是预览,
        # 观察定位用 /rtabmap/grid_map + /scan 贴合度更直观, 省 ~5% CPU。
        # 需要实时点云时用 slam_3d 模式。
    ])
