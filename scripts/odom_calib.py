#!/usr/bin/env python3
"""
odom_calib.py — 标定辅助：读取纯轮速里程计 /odom，按 Enter 报告位移和转角。

订阅 /odom (nav_msgs/Odometry)，这是 base_node 直接发布的纯 dead-reckoning，
不经过 EKF。用于 Part 1 (轮速里程计) 标定，避免 IMU 污染。

用法:
    /usr/bin/python3.12 scripts/odom_calib.py

交互:
    <Enter>     报告自上次原点至今的 dx/dy/dist/dyaw，并把当前位姿设为新原点
    r<Enter>    只重置原点，不打印
    q<Enter>    退出 (Ctrl+C 同样可以)
"""

import math
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class OdomCalib(Node):
    def __init__(self):
        super().__init__('odom_calib')
        self.create_subscription(Odometry, '/odom', self._cb, 10)
        self.latest = None
        self.origin = None
        self.lock = threading.Lock()

    def _cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        with self.lock:
            self.latest = (x, y, yaw)
            if self.origin is None:
                self.origin = (x, y, yaw)
                print(f"[ready] origin locked: x={x:+.4f} y={y:+.4f} yaw={math.degrees(yaw):+.2f}°")

    def snapshot(self):
        with self.lock:
            if self.latest is None or self.origin is None:
                return None
            x0, y0, yaw0 = self.origin
            x1, y1, yaw1 = self.latest
            return x1 - x0, y1 - y0, math.hypot(x1 - x0, y1 - y0), normalize_pi(yaw1 - yaw0)

    def reset_origin(self):
        with self.lock:
            self.origin = self.latest


def input_loop(node):
    print("\nodom_calib: 等待 /odom 第一帧 ...")
    print("命令: <Enter>=报告并重置原点 | r=只重置 | q=退出\n")
    n = 0
    while rclpy.ok():
        try:
            line = input().strip().lower()
        except EOFError:
            break
        if line == 'q':
            rclpy.shutdown()
            return
        if line == 'r':
            node.reset_origin()
            print("[reset] origin = 当前位姿")
            continue
        snap = node.snapshot()
        if snap is None:
            print("[warn] 还没有 /odom 数据，机器人启动了吗？")
            continue
        dx, dy, dist, dyaw = snap
        n += 1
        print(f"[{n:02d}] dx={dx:+.4f}m  dy={dy:+.4f}m  dist={dist:.4f}m  "
              f"dyaw={math.degrees(dyaw):+.3f}°  (raw rad={dyaw:+.5f})")
        node.reset_origin()


def main():
    rclpy.init()
    node = OdomCalib()
    threading.Thread(target=input_loop, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
