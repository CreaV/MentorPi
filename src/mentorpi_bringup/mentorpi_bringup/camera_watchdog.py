"""
Camera watchdog: supervises the Gemini 2L driver (camera.launch.py) and
auto-recovers the known USB wedge.

Failure mode (observed 2026-07-05, 2026-07-12): after a service restart the
new orbbec driver instance starts before the old one has fully released the
device; libuvc open fails ("Failed to initialize device usbEnumerator
openUsbDevice failed!") and the driver's internal reset/retry loop never
recovers — camera topics stay silent until someone runs
`usbreset 2bc5:0670` AND restarts the driver process.

Recovery sequence when /camera/depth/camera_info goes silent (or the launch
subprocess dies): SIGINT the camera launch process group, usbreset the
device, respawn the launch. Retries forever with the startup grace period
acting as backoff.
"""
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo


class CameraWatchdog(Node):
    def __init__(self):
        super().__init__('camera_watchdog')

        # camera_info 消息小 (~1KB@15Hz), 且不 lazy —— 只要 driver 在出流
        # 它就一直发, 是最便宜的活性信号 (订 raw image 会白烧回环带宽)。
        self.declare_parameter('liveness_topic', '/camera/depth/camera_info')
        # 静默多久判定卡死。夜间低光 color 可能掉到 1Hz, 但 camera_info
        # 跟随流本身, 不受曝光影响; 20s 远超任何正常间隔。
        self.declare_parameter('stall_timeout', 20.0)
        # 启动宽限: 驱动冷启动 + D2C 标定加载可到 ~20s, 再留裕量。
        # 也充当恢复失败后的重试间隔 (兜底无限重试)。
        self.declare_parameter('startup_grace', 60.0)
        self.declare_parameter('usb_id', '2bc5:0670')

        self._stall_timeout = self.get_parameter('stall_timeout').value
        self._grace = self.get_parameter('startup_grace').value
        self._usb_id = self.get_parameter('usb_id').value

        self._proc: subprocess.Popen | None = None
        self._started_at = 0.0
        self._last_msg: float | None = None
        self._restarts = 0
        self._consecutive_failures = 0

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            CameraInfo, self.get_parameter('liveness_topic').value,
            self._msg_cb, qos)

        self._start()
        self.create_timer(3.0, self._check)

    def _msg_cb(self, _msg):
        if self._last_msg is None and self._consecutive_failures:
            self._consecutive_failures = 0
        self._last_msg = time.monotonic()

    # ------- subprocess lifecycle -------

    def _start(self):
        cmd = ['ros2', 'launch', 'mentorpi_bringup', 'camera.launch.py']
        self.get_logger().info(f'starting camera: {" ".join(cmd)}')
        # New session => own process group, so SIGINT reaches the whole tree
        # (ros2 launch + component_container).
        self._proc = subprocess.Popen(cmd, start_new_session=True)
        self._started_at = time.monotonic()
        self._last_msg = None

    def _stop(self):
        proc = self._proc
        self._proc = None
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=10)
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            self.get_logger().warn('camera launch ignored SIGINT, killing')
            try:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass

    def _usbreset(self):
        try:
            res = subprocess.run(['usbreset', self._usb_id],
                                 capture_output=True, text=True, timeout=15)
            out = (res.stdout + res.stderr).strip()
            self.get_logger().info(f'usbreset {self._usb_id}: rc={res.returncode} {out}')
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error(f'usbreset failed: {exc}')

    def _recover(self, reason: str):
        self._restarts += 1
        self._consecutive_failures += 1
        self.get_logger().warn(f'camera recovery #{self._restarts}: {reason}')
        self._stop()
        self._usbreset()
        if self._consecutive_failures >= 2:
            # usbreset 不够时升级: USB authorized 软拔插 (需 root, 通过
            # sudoers 免密白名单的专用脚本, 见 scripts/gemini-usb-replug.sh)。
            # 实测 2026-07-12: openUsbDevice 死循环 usbreset 救不回,
            # authorized 0->1 一次即愈。
            self.get_logger().warn('escalating: usb authorized soft-replug')
            try:
                res = subprocess.run(
                    ['sudo', '-n', '/usr/local/sbin/gemini-usb-replug'],
                    capture_output=True, text=True, timeout=20)
                self.get_logger().info(
                    f'replug: rc={res.returncode} {(res.stdout + res.stderr).strip()}')
            except (OSError, subprocess.TimeoutExpired) as exc:
                self.get_logger().error(f'replug failed: {exc}')
        time.sleep(2.0)
        self._start()

    # ------- periodic check -------

    def _check(self):
        now = time.monotonic()
        proc = self._proc
        if proc is not None and proc.poll() is not None:
            self._recover(f'camera launch exited (rc={proc.returncode})')
            return
        if now - self._started_at < self._grace:
            return
        last = self._last_msg
        if last is None:
            self._recover(f'no camera frames within {self._grace:.0f}s of start')
        elif now - last > self._stall_timeout:
            self._recover(f'camera frames stalled for {now - last:.0f}s')

    def shutdown(self):
        self.get_logger().info('camera watchdog shutting down; stopping camera')
        self._stop()


def main(args=None):
    rclpy.init(args=args)
    node = CameraWatchdog()
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
