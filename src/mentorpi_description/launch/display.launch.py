"""
display.launch.py — 离线 URDF 预览（不连真机/sim）。

加载 mentorpi.xacro，跑 robot_state_publisher + joint_state_publisher_gui + RViz2，
用于检查 mesh 路径、TF 树结构、关节范围是否正常。

用法:
    ros2 launch mentorpi_description display.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('mentorpi_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'mentorpi.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_path]), value_type=str),
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
