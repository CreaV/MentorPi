#!/usr/bin/env python3
"""订阅 /scan，生成俯视图保存为图片，持续更新"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2
import math

class ScanViewer(Node):
    def __init__(self):
        super().__init__('scan_viewer')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, qos)
        self.img_path = '/tmp/scan_view.png'
        self.get_logger().info(f'Waiting for /scan ... image will be saved to {self.img_path}')

    def cb(self, msg):
        size = 600
        scale = 60  # pixels per meter
        img = np.zeros((size, size, 3), dtype=np.uint8)
        cx, cy = size // 2, size // 2

        # draw range rings
        for r in [1, 2, 3, 4]:
            cv2.circle(img, (cx, cy), int(r * scale), (40, 40, 40), 1)
            cv2.putText(img, f'{r}m', (cx + int(r * scale) + 3, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (80, 80, 80), 1)

        # draw cross
        cv2.line(img, (cx, 0), (cx, size), (30, 30, 30), 1)
        cv2.line(img, (0, cy), (size, cy), (30, 30, 30), 1)

        # draw robot
        cv2.circle(img, (cx, cy), 5, (0, 255, 255), -1)
        cv2.arrowedLine(img, (cx, cy), (cx, cy - 30), (0, 255, 255), 2)

        # draw scan points
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = int(cx + r * math.sin(-angle) * scale)
                y = int(cy - r * math.cos(angle) * scale)
                if 0 <= x < size and 0 <= y < size:
                    cv2.circle(img, (x, y), 2, (0, 255, 0), -1)
            angle += msg.angle_increment

        cv2.putText(img, f'Points: {len(msg.ranges)}', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(img, 'Front', (cx - 15, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        cv2.imwrite(self.img_path, img)
        self.get_logger().info('Image updated', throttle_duration_sec=2.0)

def main():
    rclpy.init()
    node = ScanViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
