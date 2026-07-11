"""
Closed-loop tests for motion_node against a fake base.

The fake base subscribes /cmd_vel and integrates a first-order velocity
response into /odometry/filtered — the same contract the real base_node+EKF
provide — so the whole action path (goal → cmd_vel streaming → odom feedback
→ completion/stop) runs without any hardware.

Run:  colcon test --packages-select mentorpi_motion
  or: pytest src/mentorpi_motion/test/test_motion_node.py  (workspace sourced)
"""
import math
import threading
import time

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from mentorpi_msgs.action import MotionPrimitive
from mentorpi_motion.motion_node import MotionNode


class FakeBase(Node):
    """Integrates cmd_vel (first-order lag) -> /odometry/filtered @ 50Hz."""

    def __init__(self, tau=0.08):
        super().__init__('fake_base')
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.cmd = Twist()
        self.tau = tau
        self.publishing = True
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.create_timer(0.02, self._step)

    def _cmd_cb(self, msg):
        self.cmd = msg

    def _step(self):
        dt = 0.02
        alpha = dt / (self.tau + dt)
        self.vx += alpha * (self.cmd.linear.x - self.vx)
        self.vy += alpha * (self.cmd.linear.y - self.vy)
        self.wz += alpha * (self.cmd.angular.z - self.wz)
        self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        self.y += (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        self.yaw += self.wz * dt
        if not self.publishing:
            return
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        self.pub.publish(odom)


class Harness:
    def __init__(self):
        rclpy.init()
        self.base = FakeBase()
        self.motion = MotionNode()
        self.client_node = Node('test_client')
        self.client = ActionClient(self.client_node, MotionPrimitive, 'motion/primitive')
        self.stop_client = self.client_node.create_client(Trigger, 'motion/stop')
        self.executor = MultiThreadedExecutor(num_threads=6)
        for n in (self.base, self.motion, self.client_node):
            self.executor.add_node(n)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        assert self.client.wait_for_server(timeout_sec=5.0)
        # Odom subscriptions are lazy (created per-goal), so there is no
        # warm-up to wait for here — _execute waits for the first odom
        # message itself.

    def shutdown(self):
        self.executor.shutdown()
        for n in (self.base, self.motion, self.client_node):
            n.destroy_node()
        rclpy.shutdown()
        self.thread.join(timeout=5.0)

    def send(self, type_, distance, max_speed=0.0, timeout=0.0):
        goal = MotionPrimitive.Goal()
        goal.type = type_
        goal.distance = float(distance)
        goal.max_speed = float(max_speed)
        goal.timeout = float(timeout)
        fut = self.client.send_goal_async(goal)
        deadline = time.monotonic() + 5.0
        while not fut.done():
            assert time.monotonic() < deadline, 'goal not accepted in time'
            time.sleep(0.01)
        return fut.result()

    def result_of(self, goal_handle, wait=30.0):
        fut = goal_handle.get_result_async()
        deadline = time.monotonic() + wait
        while not fut.done():
            assert time.monotonic() < deadline, 'no result in time'
            time.sleep(0.02)
        return fut.result().result


@pytest.fixture(scope='module')
def harness():
    h = Harness()
    yield h
    h.shutdown()


def test_forward(harness):
    gh = harness.send(MotionPrimitive.Goal.TYPE_FORWARD, 0.5)
    assert gh.accepted
    res = harness.result_of(gh)
    assert res.success, res.message
    assert abs(harness.base.x - 0.5) < 0.05
    assert abs(harness.base.y) < 0.03
    assert abs(res.traveled - 0.5) < 0.03
    time.sleep(0.5)  # base must be stopped afterwards
    assert abs(harness.base.vx) < 0.02


def test_busy_rejection(harness):
    gh = harness.send(MotionPrimitive.Goal.TYPE_FORWARD, -0.3)
    assert gh.accepted
    gh2 = harness.send(MotionPrimitive.Goal.TYPE_ROTATE, 1.0)
    assert not gh2.accepted
    res = harness.result_of(gh)
    assert res.success, res.message


def test_rotate(harness):
    yaw0 = harness.base.yaw
    gh = harness.send(MotionPrimitive.Goal.TYPE_ROTATE, math.pi / 2)
    assert gh.accepted
    res = harness.result_of(gh)
    assert res.success, res.message
    assert abs((harness.base.yaw - yaw0) - math.pi / 2) < 0.06
    assert abs(res.traveled - math.pi / 2) < 0.05


def test_strafe(harness):
    y0 = harness.base.y
    x0 = harness.base.x
    gh = harness.send(MotionPrimitive.Goal.TYPE_STRAFE, 0.3)
    assert gh.accepted
    res = harness.result_of(gh)
    assert res.success, res.message
    # displacement is along body +y at start (base may be rotated by
    # previous tests, so check magnitude of planar displacement)
    dist = math.hypot(harness.base.x - x0, harness.base.y - y0)
    assert abs(dist - 0.3) < 0.05


def test_timeout_aborts(harness):
    # Impossible goal in 1s at capped speed -> timeout abort, base stopped.
    gh = harness.send(MotionPrimitive.Goal.TYPE_FORWARD, 2.0, timeout=1.0)
    assert gh.accepted
    res = harness.result_of(gh, wait=5.0)
    assert not res.success
    assert 'timeout' in res.message
    time.sleep(0.5)
    assert abs(harness.base.vx) < 0.05


def test_stop_service(harness):
    gh = harness.send(MotionPrimitive.Goal.TYPE_FORWARD, 2.0)
    assert gh.accepted
    time.sleep(0.4)
    assert harness.stop_client.wait_for_service(timeout_sec=2.0)
    fut = harness.stop_client.call_async(Trigger.Request())
    deadline = time.monotonic() + 3.0
    while not fut.done():
        assert time.monotonic() < deadline
        time.sleep(0.02)
    assert fut.result().success
    res = harness.result_of(gh, wait=5.0)
    assert not res.success
    assert 'stop' in res.message
    time.sleep(0.5)
    assert abs(harness.base.vx) < 0.05


def test_stale_odom_aborts(harness):
    gh = harness.send(MotionPrimitive.Goal.TYPE_FORWARD, 1.5)
    assert gh.accepted
    time.sleep(0.3)
    harness.base.publishing = False
    res = harness.result_of(gh, wait=5.0)
    harness.base.publishing = True
    assert not res.success
    assert 'stale' in res.message
