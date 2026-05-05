"""
2D localization (full-stack CLI entry point). Composition of base + loc_2d.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('mentorpi_bringup')

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/home/pi/maps/my_room',
        description='Path to slam_toolbox .posegraph/.data map (no extension)',
    )

    return LaunchDescription([
        map_file_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'base.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'loc_2d.launch.py')
            ),
            launch_arguments={'map_file': LaunchConfiguration('map_file')}.items(),
        ),
    ])
