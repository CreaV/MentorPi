import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('mentorpi_bringup')

    return LaunchDescription([
        # Full robot bringup (base, lidar, teleop, static TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'mentorpi.launch.py')
            ),
        ),

        # SLAM Toolbox - online async mapping
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(bringup_dir, 'config', 'slam_toolbox_params.yaml')
            ],
        ),
    ])
