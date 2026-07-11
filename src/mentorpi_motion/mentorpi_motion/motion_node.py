"""
motion_node — bounded, odometry-closed-loop motion primitives.

The execution substrate for voice / VLA / agent control of the base:
"forward 0.5m", "rotate 90°" arrive as MotionPrimitive action goals; the
server streams /cmd_vel itself (keeping base_node's watchdog fed), measures
displacement on /odometry/filtered, and ALWAYS stops the base on completion,
cancel, timeout or stale odometry. Remote intelligence never publishes raw
cmd_vel streams.

One goal at a time; a second goal is rejected while one is active (callers
should sequence primitives). /motion/stop (std_srvs/Trigger) cancels the
active goal and zeroes cmd_vel — wire it to any "stop!" path.
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from mentorpi_msgs.action import MotionPrimitive


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class MotionNode(Node):
    def __init__(self):
        super().__init__('mentorpi_motion')

        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('default_linear_speed', 0.2)    # m/s
        self.declare_parameter('default_angular_speed', 0.8)   # rad/s
        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('linear_accel', 0.5)            # m/s^2 ramp
        self.declare_parameter('angular_accel', 2.0)           # rad/s^2
        self.declare_parameter('min_linear_speed', 0.05)       # stiction floor
        self.declare_parameter('min_angular_speed', 0.15)
        self.declare_parameter('linear_tolerance', 0.01)       # m
        self.declare_parameter('angular_tolerance', 0.02)      # rad (~1.1 deg)
        self.declare_parameter('max_linear_distance', 3.0)     # m, per-goal cap
        self.declare_parameter('max_angular_distance', 12.6)   # rad (~2 turns)
        self.declare_parameter('odom_timeout', 0.5)            # s, stale odom abort
        self.declare_parameter('control_rate', 20.0)           # Hz
        # 航向保持 (forward/strafe): 麦轮开环直行会因电机不均匀画弧
        # (实测 1m 明显偏左), 用 EKF yaw (陀螺仪) 做 P 修正。
        self.declare_parameter('heading_kp', 2.0)              # (rad/s)/rad
        self.declare_parameter('heading_max_correction', 0.5)  # rad/s

        p = self.get_parameter
        self._default_speed = {
            MotionPrimitive.Goal.TYPE_FORWARD: p('default_linear_speed').value,
            MotionPrimitive.Goal.TYPE_STRAFE: p('default_linear_speed').value,
            MotionPrimitive.Goal.TYPE_ROTATE: p('default_angular_speed').value,
        }
        self._max_speed = {
            MotionPrimitive.Goal.TYPE_FORWARD: p('max_linear_speed').value,
            MotionPrimitive.Goal.TYPE_STRAFE: p('max_linear_speed').value,
            MotionPrimitive.Goal.TYPE_ROTATE: p('max_angular_speed').value,
        }
        self._accel = {
            MotionPrimitive.Goal.TYPE_FORWARD: p('linear_accel').value,
            MotionPrimitive.Goal.TYPE_STRAFE: p('linear_accel').value,
            MotionPrimitive.Goal.TYPE_ROTATE: p('angular_accel').value,
        }
        self._min_speed = {
            MotionPrimitive.Goal.TYPE_FORWARD: p('min_linear_speed').value,
            MotionPrimitive.Goal.TYPE_STRAFE: p('min_linear_speed').value,
            MotionPrimitive.Goal.TYPE_ROTATE: p('min_angular_speed').value,
        }
        self._tolerance = {
            MotionPrimitive.Goal.TYPE_FORWARD: p('linear_tolerance').value,
            MotionPrimitive.Goal.TYPE_STRAFE: p('linear_tolerance').value,
            MotionPrimitive.Goal.TYPE_ROTATE: p('angular_tolerance').value,
        }
        self._max_distance = {
            MotionPrimitive.Goal.TYPE_FORWARD: p('max_linear_distance').value,
            MotionPrimitive.Goal.TYPE_STRAFE: p('max_linear_distance').value,
            MotionPrimitive.Goal.TYPE_ROTATE: p('max_angular_distance').value,
        }
        self._odom_timeout = p('odom_timeout').value
        self._control_dt = 1.0 / p('control_rate').value
        self._heading_kp = p('heading_kp').value
        self._heading_max_corr = p('heading_max_correction').value

        # 里程计发散熔断: EKF (/odometry/filtered) 曾在串口过流掉线后
        # 发散出 4.4m 假位移(2026-07-05), goal 追着假目标让车乱动。
        # 对照原始 /odom (纯指令积分, 不经 EKF), 两者位移分家超阈值
        # 立即停车。阈值放宽到远超正常打滑差异, 只拦真发散。
        self.declare_parameter('raw_odom_topic', '/odom')
        self.declare_parameter('divergence_linear', 0.5)   # m
        self.declare_parameter('divergence_angular', 1.5)  # rad
        self._div_linear = p('divergence_linear').value
        self._div_angular = p('divergence_angular').value

        # Latest odom sample (guarded by _odom_lock)
        self._odom_lock = threading.Lock()
        self._odom = None            # (x, y, yaw)
        self._odom_mono = None       # time.monotonic() of last sample
        self._raw_odom = None        # (x, y, yaw) from raw_odom_topic

        self._active_lock = threading.Lock()
        self._active = False

        cb_group = ReentrantCallbackGroup()
        self._cb_group = cb_group
        self._odom_topic = p('odom_topic').value
        self._raw_odom_topic = p('raw_odom_topic').value
        self._cmd_pub = self.create_publisher(Twist, p('cmd_vel_topic').value, 10)

        # 里程计订阅是懒加载的: 只在 goal 执行期间订阅。常驻订阅 2 路
        # 50Hz 会让 rclpy 执行器空闲时也烧 ~38% CPU (Pi 5 实测), 抢走
        # SLAM 的算力; motion 空闲占比 99%, 不值得。
        self._sub_handles: list = []

        self._server = ActionServer(
            self, MotionPrimitive, 'motion/primitive',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=cb_group,
        )
        self._stop_srv = self.create_service(
            Trigger, 'motion/stop', self._stop_cb, callback_group=cb_group)
        self._stop_requested = False

        self.get_logger().info('motion primitives ready (motion/primitive, motion/stop)')

    # ------- odom -------

    def _odom_cb(self, msg: Odometry):
        pose = msg.pose.pose
        with self._odom_lock:
            self._odom = (pose.position.x, pose.position.y,
                          yaw_from_quat(pose.orientation))
            self._odom_mono = time.monotonic()

    def _raw_odom_cb(self, msg: Odometry):
        pose = msg.pose.pose
        with self._odom_lock:
            self._raw_odom = (pose.position.x, pose.position.y,
                              yaw_from_quat(pose.orientation))

    def _odom_snapshot(self):
        with self._odom_lock:
            return self._odom, self._odom_mono

    def _raw_snapshot(self):
        with self._odom_lock:
            return self._raw_odom

    def _start_subs(self) -> None:
        if self._sub_handles:
            return
        self._sub_handles = [
            self.create_subscription(
                Odometry, self._odom_topic, self._odom_cb, 20,
                callback_group=self._cb_group),
            self.create_subscription(
                Odometry, self._raw_odom_topic, self._raw_odom_cb, 20,
                callback_group=self._cb_group),
        ]

    def _stop_subs(self) -> None:
        for s in self._sub_handles:
            self.destroy_subscription(s)
        self._sub_handles = []
        with self._odom_lock:
            self._odom = None
            self._odom_mono = None
            self._raw_odom = None

    # ------- goal admission -------

    def _goal_cb(self, goal: MotionPrimitive.Goal) -> GoalResponse:
        if goal.type not in self._default_speed:
            self.get_logger().warn(f'rejecting goal: unknown type "{goal.type}"')
            return GoalResponse.REJECT
        if goal.distance == 0.0 or not math.isfinite(goal.distance):
            self.get_logger().warn('rejecting goal: distance must be nonzero finite')
            return GoalResponse.REJECT
        if abs(goal.distance) > self._max_distance[goal.type]:
            self.get_logger().warn(
                f'rejecting goal: |distance| {abs(goal.distance):.2f} exceeds cap '
                f'{self._max_distance[goal.type]:.2f} for {goal.type}')
            return GoalResponse.REJECT
        # 里程计新鲜度改到 _execute 开头检查(懒订阅后, 空闲时本来就没有
        # odom 数据, 在这里拒绝会拒掉一切 goal)。
        with self._active_lock:
            if self._active:
                self.get_logger().warn('rejecting goal: a primitive is already running')
                return GoalResponse.REJECT
            self._active = True
        return GoalResponse.ACCEPT

    # ------- stop service -------

    def _stop_cb(self, _req, res):
        self._stop_requested = True
        self._publish_stop()
        res.success = True
        res.message = 'stop requested'
        return res

    # ------- execution -------

    def _publish_stop(self):
        try:
            self._cmd_pub.publish(Twist())
        except Exception:
            # Only fails when the rclpy context is already torn down
            # (shutdown race) — the base's own cmd_vel watchdog stops the
            # motors in that case.
            pass

    def _traveled(self, goal_type: str, start, current) -> float:
        x0, y0, yaw0 = start
        x, y, _yaw = current
        if goal_type == MotionPrimitive.Goal.TYPE_FORWARD:
            return (x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)
        if goal_type == MotionPrimitive.Goal.TYPE_STRAFE:
            return -(x - x0) * math.sin(yaw0) + (y - y0) * math.cos(yaw0)
        raise ValueError(goal_type)

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = MotionPrimitive.Result()
        self._stop_requested = False

        speed_cap = self._max_speed[goal.type]
        speed = goal.max_speed if goal.max_speed > 0.0 else self._default_speed[goal.type]
        speed = min(speed, speed_cap)
        accel = self._accel[goal.type]
        tol = self._tolerance[goal.type]
        min_speed = self._min_speed[goal.type]
        timeout = goal.timeout if goal.timeout > 0.0 else \
            (abs(goal.distance) / speed) * 2.0 + 2.0

        rotate = goal.type == MotionPrimitive.Goal.TYPE_ROTATE

        # 懒订阅: 现在才挂上里程计, 等第一帧 (最多 odom_timeout*2)。
        self._start_subs()
        wait_deadline = time.monotonic() + self._odom_timeout * 2.0
        start = None
        while time.monotonic() < wait_deadline:
            start, _ = self._odom_snapshot()
            if start is not None:
                break
            time.sleep(0.02)
        if start is None:
            self._stop_subs()
            with self._active_lock:
                self._active = False
            result.success = False
            result.message = 'no odometry available'
            result.traveled = 0.0
            self._finalize(goal_handle, 'abort', result)
            return result

        prev_yaw = start[2]
        traveled = 0.0
        # Divergence guard baseline (None when raw odom absent, e.g. tests).
        raw_start = self._raw_snapshot()
        raw_prev_yaw = raw_start[2] if raw_start else 0.0
        raw_traveled = 0.0
        v_cmd = 0.0
        t0 = time.monotonic()
        last_feedback = 0.0
        outcome = None  # None while running, else (success, message)

        try:
            while rclpy.ok():
                now = time.monotonic()
                odom, odom_mono = self._odom_snapshot()

                if self._stop_requested:
                    outcome = (False, 'stopped by motion/stop')
                    break
                if goal_handle.is_cancel_requested:
                    outcome = (False, 'canceled')
                    break
                if now - odom_mono > self._odom_timeout:
                    outcome = (False, f'odometry stale > {self._odom_timeout}s')
                    break
                if now - t0 > timeout:
                    outcome = (False, f'timeout after {timeout:.1f}s')
                    break

                # Measure displacement along the commanded axis.
                if rotate:
                    traveled += wrap_pi(odom[2] - prev_yaw)
                    prev_yaw = odom[2]
                else:
                    traveled = self._traveled(goal.type, start, odom)
                remaining = goal.distance - traveled

                # Divergence guard: EKF vs raw cmd-integration odometry.
                raw = self._raw_snapshot()
                if raw_start is None and raw is not None:
                    # 懒订阅下 raw 首帧可能比 filtered 晚到, 迟捕获基线。
                    raw_start = raw
                    raw_prev_yaw = raw[2]
                if raw_start is not None and raw is not None:
                    if rotate:
                        raw_traveled += wrap_pi(raw[2] - raw_prev_yaw)
                        raw_prev_yaw = raw[2]
                        diverged = abs(traveled - raw_traveled) > self._div_angular
                    else:
                        raw_traveled = self._traveled(goal.type, raw_start, raw)
                        diverged = abs(traveled - raw_traveled) > self._div_linear
                    if diverged:
                        outcome = (False,
                                   f'odometry divergence: ekf {traveled:+.2f} vs '
                                   f'raw {raw_traveled:+.2f} — EKF unstable?')
                        break

                if abs(remaining) <= tol:
                    outcome = (True, 'done')
                    break

                # Trapezoid: accel-limited ramp up, sqrt-rule ramp down.
                direction = 1.0 if remaining > 0.0 else -1.0
                v_target = min(speed, math.sqrt(2.0 * accel * abs(remaining)))
                v_target = max(v_target, min_speed)
                v_mag = min(abs(v_cmd) + accel * self._control_dt, v_target)
                v_cmd = direction * v_mag

                cmd = Twist()
                if goal.type == MotionPrimitive.Goal.TYPE_FORWARD:
                    cmd.linear.x = v_cmd
                elif goal.type == MotionPrimitive.Goal.TYPE_STRAFE:
                    cmd.linear.y = v_cmd
                else:
                    cmd.angular.z = v_cmd
                if not rotate:
                    # Heading hold: P-correct back to the yaw captured at
                    # goal start, so open-loop motor imbalance can't arc.
                    yaw_err = wrap_pi(start[2] - odom[2])
                    corr = self._heading_kp * yaw_err
                    cmd.angular.z = max(-self._heading_max_corr,
                                        min(self._heading_max_corr, corr))
                self._cmd_pub.publish(cmd)

                if now - last_feedback > 0.2:
                    last_feedback = now
                    fb = MotionPrimitive.Feedback()
                    fb.traveled = float(traveled)
                    fb.progress = float(max(0.0, min(1.0, traveled / goal.distance)))
                    goal_handle.publish_feedback(fb)

                time.sleep(self._control_dt)

            if outcome is None:  # rclpy shut down mid-run
                outcome = (False, 'interrupted')
        finally:
            # Whatever happened, leave the base stopped.
            self._publish_stop()
            self._stop_subs()  # 懒订阅: goal 结束即退订, 空闲不吃 CPU
            with self._active_lock:
                self._active = False

        success, message = outcome
        result.success = success
        result.message = message
        result.traveled = float(traveled)
        if success:
            self._finalize(goal_handle, 'succeed', result)
            self.get_logger().info(
                f'{goal.type} {goal.distance:+.3f} done (traveled {traveled:+.3f})')
        elif goal_handle.is_cancel_requested:
            self._finalize(goal_handle, 'canceled', result)
            self.get_logger().info(f'{goal.type} canceled at {traveled:+.3f}')
        else:
            self._finalize(goal_handle, 'abort', result)
            self.get_logger().warn(f'{goal.type} aborted: {message} (at {traveled:+.3f})')
        return result

    @staticmethod
    def _finalize(goal_handle, method_name: str, result) -> None:
        """Set terminal state WITH the result attached.

        Jazzy rclpy fills the result future the moment succeed()/abort()/
        canceled() runs — with an empty default Result unless it is passed
        here. Relying on the execute-callback return value races against
        already-pending client result requests (observed losing under load).
        Older rclpy takes no argument; fall back for compatibility.
        """
        method = getattr(goal_handle, method_name)
        try:
            method(result)
        except TypeError:
            method()


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
