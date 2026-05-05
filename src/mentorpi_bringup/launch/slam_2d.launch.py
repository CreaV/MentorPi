"""
2D SLAM mode-only launch. Assumes base.launch.py is already running
(provides /scan, EKF odom TF). Started/stopped by mentorpi_supervisor.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('mentorpi_bringup')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[os.path.join(bringup_dir, 'config', 'slam_toolbox_params.yaml')],
        ),
    ])
