import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from mentorpi_msgs.msg import Gimbal

DEADZONE = 0.1
RB_BUTTON = 7  # R1/RB 在北通手柄的 buttons 索引

def apply_deadzone(value, deadzone=DEADZONE):
    if abs(value) < deadzone:
        return 0.0
    return value

class MentorPiTeleop(Node):
    def __init__(self):
        super().__init__('mentorpi_teleop')

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gimbal_pub = self.create_publisher(Gimbal, '/gimbal/cmd', 10)

        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.5)

        self.get_logger().info("Teleop node started (mecanum mode). Hold RB for gimbal control.")

    def joy_callback(self, msg):
        if len(msg.axes) < 4:
            return

        max_l = self.get_parameter('max_linear_vel').value
        max_a = self.get_parameter('max_angular_vel').value

        rb_held = len(msg.buttons) > RB_BUTTON and msg.buttons[RB_BUTTON]

        twist = Twist()
        gimbal = Gimbal()

        twist.linear.x = apply_deadzone(msg.axes[1]) * max_l
        twist.linear.y = apply_deadzone(msg.axes[0]) * max_l

        if rb_held:
            # RB 按住: 右摇杆控制云台, 底盘不旋转
            twist.angular.z = 0.0
            gimbal.pitch = 90.0 + apply_deadzone(msg.axes[3]) * 90.0
            gimbal.yaw = 90.0 - apply_deadzone(msg.axes[2]) * 90.0
        else:
            # 松开: 右摇杆X控制旋转, 云台自动回中
            twist.angular.z = apply_deadzone(msg.axes[2]) * max_a
            gimbal.pitch = 90.0
            gimbal.yaw = 90.0

        self.cmd_vel_pub.publish(twist)
        self.gimbal_pub.publish(gimbal)

def main(args=None):
    rclpy.init(args=args)
    node = MentorPiTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
