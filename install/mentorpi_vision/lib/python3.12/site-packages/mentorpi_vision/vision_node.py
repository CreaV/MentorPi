import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class MentorPiVision(Node):
    def __init__(self):
        super().__init__('mentorpi_vision')
        
        # 发布图像
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # 定时器：捕获图像
        self.timer = self.create_timer(0.033, self.timer_callback) # 30 FPS
        
        # 尝试打开相机
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera 0")

        self.get_logger().info("Vision node started.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # 可以在此处加入云台位置叠加到图像的逻辑
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MentorPiVision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
