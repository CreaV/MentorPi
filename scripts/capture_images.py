import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.color_msg = None
        self.depth_msg = None
        
        # Subscribe to standard Orbbec ROS2 topics
        self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        
        self.get_logger().info('Waiting for image messages...')

    def color_callback(self, msg):
        self.color_msg = msg

    def depth_callback(self, msg):
        self.depth_msg = msg

def main():
    rclpy.init()
    node = ImageSaver()
    
    start_time = time.time()
    timeout = 15  # 15 seconds timeout
    
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1)
        if node.color_msg and node.depth_msg:
            break
        if time.time() - start_time > timeout:
            print("Timeout waiting for images!")
            break

    if node.color_msg:
        try:
            cv_color = node.bridge.imgmsg_to_cv2(node.color_msg, desired_encoding='bgr8')
            cv2.imwrite('test_rgb.png', cv_color)
            print("Saved test_rgb.png")
        except Exception as e:
            print(f"Error saving color image: {e}")

    if node.depth_msg:
        try:
            # Depth is usually 16-bit millimeters (uint16)
            cv_depth = node.bridge.imgmsg_to_cv2(node.depth_msg, desired_encoding='passthrough')
            
            # 1. Mask out zero (invalid) values
            mask = cv_depth > 0
            
            # 2. Limit range to 0.5m - 3.0m (500mm - 3000mm) for better contrast
            # This is common for indoor robot navigation
            depth_clipped = np.clip(cv_depth, 500, 3000)
            
            # 3. Normalize only the valid range
            # We normalize 500-3000 to 0-255
            depth_norm = ((depth_clipped - 500) / (3000 - 500) * 255).astype(np.uint8)
            
            # 4. Apply a colormap that shows more detail
            # COLORMAP_BONE or COLORMAP_VIRIDIS are often clearer than JET for depth
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            
            # Set invalid areas (zero) to pure black
            depth_color[~mask] = 0
            
            cv2.imwrite('test_depth_v2.png', depth_color)
            print("Saved test_depth_v2.png (improved visualization)")
        except Exception as e:
            print(f"Error saving depth image: {e}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
