"""
3D SLAM mode-only launch. Adds rtabmap + colored point cloud on top of the
already-running base (which provides EKF odom and the Gemini 2 RGB-D
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
from launch_ros.parameter_descriptions import ParameterValue

from mentorpi_bringup import rtabmap_params as rp


def generate_launch_description():
    # 增量建图(多会话)说明: rtabmap 对已存在的 database_path 是追加式 —
    # 同一个 db 再次进入 slam_3d 即"续图", 新旧会话靠回环/近邻链接合并;
    # 想从零建新图, 传一个新文件名即可(不必手动备份旧库)。
    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value='~/rtabmap_maps/rtabmap.db',
        description='Path to RTAB-Map database file (existing = continue mapping)')
    database_path = LaunchConfiguration('database_path')

    # 续图时建议开 load_all_nodes:=true —— 把旧图全部节点载入工作内存,
    # 新会话一开始就能对旧图重定位, 立即合并坐标系, 而不是先在自己的
    # 会话里漂移、等撞上回环才归位。代价是加载耗时 + 内存(484 节点的
    # 库约十几秒), 所以默认关。
    load_all_nodes_arg = DeclareLaunchArgument(
        'load_all_nodes', default_value='false',
        description='Load all old-map nodes into WM at start (better session merging)')
    load_all_nodes = LaunchConfiguration('load_all_nodes')

    return LaunchDescription([
        database_path_arg,
        load_all_nodes_arg,

        # Pair RGB + depth (+ color camera_info) into one RGBDImage so
        # rtabmap only has to sync it with the scan, with a bounded window.
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            parameters=[rp.RGBD_SYNC_PARAMS],
            remappings=rp.RGBD_SYNC_REMAPPINGS,
        ),

        # rtabmap (mapping + loop closure)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                **rp.slam_params(),
                'database_path': database_path,
                'Mem/IncrementalMemory': 'true',
                # rtabmap 的参数全是字符串类型; LaunchConfiguration 直接传
                # 会被 YAML 解析成 bool -> rtabmap 启动即 abort (实测 2026-07-12)。
                'Mem/InitWMWithAllNodes': ParameterValue(load_all_nodes, value_type=str),
            }],
            remappings=rp.REMAPPINGS,
            # 关闭时给 rtabmap 足够时间落库 (见 rtabmap_params 注释)。
            sigterm_timeout=rp.RTABMAP_SIGTERM_TIMEOUT,
            sigkill_timeout=rp.RTABMAP_SIGKILL_TIMEOUT,
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
