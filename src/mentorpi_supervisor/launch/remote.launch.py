"""
Remote operation entry point.

Brings up:
  - base.launch.py             (always-on hardware + Gemini 2L)
  - mentorpi_supervisor        (mode switcher: /mode/set, /mode/list_maps, /mode/status)
  - foxglove_bridge      :8765 (ws -- desktop Foxglove Studio clients)
  - rosbridge_websocket  :9090 (ws -- AIRE voice skill, live_rerun.py, mobile SPA)
  - web_video_server     :8081 (http MJPEG -- browser <img> tag, needs enable_mobile_spa)
  - http.server          :8000 (http static -- the SPA itself, needs enable_mobile_spa)

Mobile/tablet flow:
  Browser -> http://<robot-ip>:8000/  -> SPA pulls assets, opens
  ws://<robot-ip>:9090 + http://<robot-ip>:8081/stream?topic=...

Desktop Foxglove Studio:
  ws://<robot-ip>:8765 + import foxglove_layout/mentorpi.json
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('mentorpi_bringup')
    supervisor_share = get_package_share_directory('mentorpi_supervisor')
    web_dir = os.path.join(supervisor_share, 'web')

    enable_mobile_spa = LaunchConfiguration('enable_mobile_spa')

    return LaunchDescription([
        # Foxglove Studio is the primary client and runs by default.
        # rosbridge (:9090) also runs by default: it is no longer just the
        # mobile SPA transport -- the AIRE voice skill (robot.* tools) and
        # scripts/live_rerun.py both depend on it.
        # The rest of the mobile SPA stack (web_video_server + static
        # http.server) stays dormant unless enabled with
        # `enable_mobile_spa:=true`.
        DeclareLaunchArgument('enable_mobile_spa', default_value='false',
            description='Start web_video_server + SPA HTTP server (mobile client)'),

        DeclareLaunchArgument('foxglove_port', default_value='8765',
            description='foxglove_bridge websocket port'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090',
            description='rosbridge_websocket port (mobile SPA)'),
        DeclareLaunchArgument('video_port', default_value='8081',
            description='web_video_server HTTP port (MJPEG, mobile SPA)'),
        DeclareLaunchArgument('spa_port', default_value='8000',
            description='HTTP port serving the static SPA'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'base.launch.py')
            ),
        ),

        Node(
            package='mentorpi_supervisor',
            executable='supervisor_node',
            name='mentorpi_supervisor',
            output='screen',
        ),

        # ---- bridges ----

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('foxglove_port'),
                'address': '0.0.0.0',
                'tls': False,
                # 每客户端发送队列上限。10MB 在 WiFi 吞吐跟不上订阅流量时
                # 会积成"几分钟延迟"的陈旧队列(实测); 1MB 让桥主动丢旧帧,
                # 延迟封顶在 ~1 秒级, 画面永远是新鲜的。
                'send_buffer_limit': 1000000,
                'use_compression': False,
                'topic_whitelist': ['.*'],
                'service_whitelist': ['.*'],
                'param_whitelist': ['.*'],
                'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe',
                                 'services', 'connectionGraph', 'assets'],
            }],
        ),

        # 远程查看用的低频图像流 (2Hz)。15fps 压缩流在 2.4G WiFi 上供不起
        # (夜间高增益噪声帧更肥), 订阅端只需要预览帧率 —— 源头节流,
        # live_rerun 默认订这个。需要 ros-jazzy-topic-tools。
        Node(
            package='topic_tools',
            executable='throttle',
            name='viewer_image_throttle',
            output='screen',
            arguments=['messages', '/camera/color/image_raw/compressed',
                       '2.0', '/viewer/color_compressed'],
        ),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            # Force /usr/bin/python3 (system 3.12) -- conda's 3.13 in PATH would
            # mismatch jazzy's rclpy C extension and crash at import.
            prefix='/usr/bin/python3',
            parameters=[{
                'port': LaunchConfiguration('rosbridge_port'),
                'address': '0.0.0.0',
                'max_message_size': 10000000,
                'call_services_in_new_thread': True,
            }],
        ),

        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('video_port'),
                'address': '0.0.0.0',
                'default_stream_type': 'mjpeg',
            }],
            condition=IfCondition(enable_mobile_spa),
        ),

        # ---- static SPA ----

        ExecuteProcess(
            cmd=[
                '/usr/bin/python3', '-m', 'http.server',
                LaunchConfiguration('spa_port'),
                '--bind', '0.0.0.0',
                '--directory', web_dir,
            ],
            output='screen',
            name='spa_http_server',
            condition=IfCondition(enable_mobile_spa),
        ),
    ])
