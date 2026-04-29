import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('mentorpi_bringup')

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/home/pi/maps/my_room',
        description='Path to slam_toolbox .posegraph/.data map (without extension)',
    )

    return LaunchDescription([
        map_file_arg,

        # Full robot bringup (base, lidar, teleop, static TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'mentorpi.launch.py')
            ),
        ),

        # SLAM Toolbox - localization mode
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(bringup_dir, 'config', 'slam_toolbox_localization_params.yaml'),
                {'map_file_name': LaunchConfiguration('map_file')},
            ],
        ),
    ])
