"""
3D SLAM (full-stack CLI entry point). Composition of base + slam_3d.

For interactive remote operation use remote.launch.py + supervisor instead;
this launch is for direct CLI usage. Pass database_path as a launch arg.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('mentorpi_bringup')

    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value='~/rtabmap_maps/rtabmap.db',
        description='Path to RTAB-Map database file')

    return LaunchDescription([
        database_path_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'base.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'slam_3d.launch.py')
            ),
            launch_arguments={'database_path': LaunchConfiguration('database_path')}.items(),
        ),
    ])
