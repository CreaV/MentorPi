"""
Supervisor node: manages mutually-exclusive SLAM/localization modes on top of
an always-running base.launch.py.

Exposes:
  /mode/set        (mentorpi_msgs/srv/SetMode)
  /mode/list_maps  (mentorpi_msgs/srv/ListMaps)
  /mode/status     (std_msgs/String, transient_local)  -- current mode name

Modes:
  idle      no extra launch
  slam_2d   ros2 launch mentorpi_bringup slam_2d.launch.py
  slam_3d   ros2 launch mentorpi_bringup slam_3d.launch.py [database_path:=...]
  loc_2d    ros2 launch mentorpi_bringup loc_2d.launch.py map_file:=...
  loc_3d    ros2 launch mentorpi_bringup loc_3d.launch.py [database_path:=...]
"""
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

from mentorpi_msgs.srv import SetMode, ListMaps
from mentorpi_msgs.msg import Buzzer


VALID_MODES = {'idle', 'slam_2d', 'slam_3d', 'loc_2d', 'loc_3d'}

MAP_DIRS = {
    'loc_2d': Path.home() / 'maps',
    'slam_3d': Path.home() / 'rtabmap_maps',
    'loc_3d': Path.home() / 'rtabmap_maps',
}

DEFAULT_RTABMAP_DB = str(Path.home() / 'rtabmap_maps' / 'rtabmap.db')

# Startup chime: 3 short beeps so the user knows base_node is up & supervisor
# is ready to take service calls.
CHIME_FREQ = 1900
CHIME_ON_S = 0.08
CHIME_OFF_S = 0.10
CHIME_REPEAT = 3
CHIME_DISCOVERY_TIMEOUT_S = 30


class SupervisorNode(Node):
    def __init__(self):
        super().__init__('mentorpi_supervisor')

        self.declare_parameter('enable_startup_chime', True)
        self._enable_chime = self.get_parameter('enable_startup_chime').value

        # State (guarded by self._lock)
        self._lock = threading.Lock()
        self._proc: subprocess.Popen | None = None
        self._current_mode: str = 'idle'
        self._busy: bool = False  # True during start/stop transitions

        # Latched mode-status publisher (transient_local so late subscribers see it)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(String, '/mode/status', latched_qos)

        # Buzzer publisher used for the startup "ready" chime.
        self._buzzer_pub = self.create_publisher(Buzzer, '/buzzer', 1)
        self._chime_deadline = time.monotonic() + CHIME_DISCOVERY_TIMEOUT_S
        self._chime_timer = self.create_timer(0.5, self._try_chime) if self._enable_chime else None

        # Services
        self._set_mode_srv = self.create_service(SetMode, '/mode/set', self._set_mode_cb)
        self._list_maps_srv = self.create_service(ListMaps, '/mode/list_maps', self._list_maps_cb)

        # Watchdog: detect mode subprocess crash
        self._watchdog_timer = self.create_timer(2.0, self._watchdog)

        self._publish_status()
        self.get_logger().info('Supervisor ready (mode=idle)')

    def _try_chime(self):
        """Poll for /buzzer subscriber, publish chime once, then stop."""
        if self._buzzer_pub.get_subscription_count() > 0:
            msg = Buzzer(
                freq=CHIME_FREQ,
                on_time=CHIME_ON_S,
                off_time=CHIME_OFF_S,
                repeat=CHIME_REPEAT,
            )
            self._buzzer_pub.publish(msg)
            self.get_logger().info('startup chime sent')
            self._chime_timer.cancel()
            self._chime_timer = None
            return
        if time.monotonic() > self._chime_deadline:
            self.get_logger().warn(
                f'no /buzzer subscriber after {CHIME_DISCOVERY_TIMEOUT_S}s; skipping chime')
            self._chime_timer.cancel()
            self._chime_timer = None

    # ------- public service callbacks -------

    def _set_mode_cb(self, request: SetMode.Request, response: SetMode.Response):
        target = request.mode.strip() or 'idle'

        if target not in VALID_MODES:
            response.success = False
            response.message = f'unknown mode "{target}", valid: {sorted(VALID_MODES)}'
            response.current_mode = self._current_mode
            return response

        if target == 'loc_2d' and not request.map_file:
            response.success = False
            response.message = 'loc_2d requires map_file (path without .posegraph extension)'
            response.current_mode = self._current_mode
            return response

        with self._lock:
            if self._busy:
                response.success = False
                response.message = 'supervisor busy with previous transition, retry shortly'
                response.current_mode = self._current_mode
                return response
            if target == self._current_mode and target == 'idle':
                response.success = True
                response.message = 'already idle'
                response.current_mode = self._current_mode
                return response
            self._busy = True

        try:
            self._stop_active()
            if target != 'idle':
                self._start_mode(target, request.map_file, request.database_path,
                                 request.load_all_nodes)
            with self._lock:
                self._current_mode = target
            self._publish_status()
            response.success = True
            response.message = f'switched to {target}'
        except Exception as exc:
            self.get_logger().error(f'mode switch failed: {exc}')
            with self._lock:
                self._current_mode = 'idle'
            self._publish_status()
            response.success = False
            response.message = f'failed: {exc}'
        finally:
            with self._lock:
                self._busy = False

        response.current_mode = self._current_mode
        return response

    def _list_maps_cb(self, request: ListMaps.Request, response: ListMaps.Response):
        mode = request.mode.strip()
        directory = MAP_DIRS.get(mode)
        if directory is None:
            response.maps = []
            return response
        if not directory.is_dir():
            response.maps = []
            return response

        if mode == 'loc_2d':
            # slam_toolbox saves <name>.posegraph + <name>.data; expose <name>.
            names = sorted({p.stem for p in directory.glob('*.posegraph')})
            response.maps = [str(directory / n) for n in names]
        elif mode in ('slam_3d', 'loc_3d'):
            response.maps = sorted(str(p) for p in directory.glob('*.db'))
        else:
            response.maps = []
        return response

    # ------- subprocess lifecycle -------

    def _start_mode(self, mode: str, map_file: str, database_path: str,
                    load_all_nodes: bool = False):
        cmd = ['ros2', 'launch', 'mentorpi_bringup', f'{mode}.launch.py']
        if mode == 'loc_2d':
            cmd.append(f'map_file:={map_file}')
        elif mode in ('slam_3d', 'loc_3d'):
            cmd.append(f'database_path:={database_path or DEFAULT_RTABMAP_DB}')
            # 增量续图: 旧图节点全载入 WM, 开局即重定位合并 (勿漂移等回环)。
            # loc_3d 的 launch 里本来就是 InitWMWithAllNodes=true, 只对
            # slam_3d 有意义。
            if mode == 'slam_3d' and load_all_nodes:
                cmd.append('load_all_nodes:=true')

        self.get_logger().info(f'launching: {" ".join(cmd)}')
        # start_new_session=True puts the launch and all its children in a new
        # process group so we can SIGINT them all at once.
        self._proc = subprocess.Popen(cmd, start_new_session=True)
        # Brief settle time so callers don't immediately see "still starting" gaps.
        time.sleep(0.5)
        if self._proc.poll() is not None:
            raise RuntimeError(f'launch exited immediately with code {self._proc.returncode}')

    def _stop_active(self):
        proc = self._proc
        if proc is None:
            return
        if proc.poll() is not None:
            self._proc = None
            return

        self.get_logger().info(f'stopping mode {self._current_mode} (pid={proc.pid})')
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            self._proc = None
            return

        # Give ros2 launch time to shut nodes down gracefully. rtabmap
        # flushes its whole database on shutdown — a 160MB db takes well
        # over 10s on the Pi's SD card, and killing it mid-save corrupts
        # the visual word dictionary (loadWordsQuery 0 words, relocation
        # permanently broken — observed 2026-07-05). 3D modes get a long
        # grace period; SIGKILL only as the very last resort.
        sigint_grace = 90 if self._current_mode in ('slam_3d', 'loc_3d') else 10
        try:
            proc.wait(timeout=sigint_grace)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                f'launch did not exit {sigint_grace}s after SIGINT, sending SIGTERM')
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=15)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                self.get_logger().error('launch unresponsive, sending SIGKILL')
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait(timeout=5)
        self._proc = None

    def _watchdog(self):
        with self._lock:
            if self._busy or self._proc is None:
                return
            if self._proc.poll() is None:
                return
            # Subprocess died unexpectedly.
            rc = self._proc.returncode
            mode = self._current_mode
            self._proc = None
            self._current_mode = 'idle'
        self.get_logger().error(f'mode {mode} subprocess exited unexpectedly (rc={rc}); reverting to idle')
        self._publish_status()

    def _publish_status(self):
        msg = String()
        msg.data = self._current_mode
        self._status_pub.publish(msg)

    # ------- shutdown -------

    def shutdown(self):
        self.get_logger().info('supervisor shutting down; stopping active mode if any')
        try:
            self._stop_active()
        except Exception as exc:
            self.get_logger().error(f'error stopping active mode at shutdown: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
