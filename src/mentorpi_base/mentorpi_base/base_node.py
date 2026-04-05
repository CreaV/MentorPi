import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from mentorpi_msgs.msg import Gimbal, MotorStatus
from tf2_ros import TransformBroadcaster
import serial
import struct
import math


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q

# 官方 CRC8 查表法
crc8_table = [
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
]

def checksum_crc8(data):
    check = 0
    for b in data:
        check = crc8_table[check ^ b]
    return check & 0xFF

FUNC_MOTOR = 3
FUNC_PWM_SERVO = 4

class MentorPiBase(Node):
    def __init__(self):
        super().__init__('mentorpi_base')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(None, baud, timeout=0.1)
            self.ser.rts = False
            self.ser.dtr = False
            self.ser.setPort(port)
            self.ser.open()
            self.get_logger().info(f"Connected to RRCLite: {port} at {baud}")
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")
            self.ser = None

        # 初始化：停止所有电机
        self.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.gimbal_sub = self.create_subscription(Gimbal, '/gimbal/cmd', self.gimbal_callback, 10)

        # Odometry state (command-based dead-reckoning)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.last_odom_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.02, self.odom_timer_callback)  # 50 Hz

    def send_packet(self, func, data):
        if not self.ser:
            return
        buf = [0xAA, 0x55, int(func)]
        buf.append(len(data))
        buf.extend(data)
        buf.append(checksum_crc8(bytes(buf[2:])))
        self.ser.write(bytes(buf))

    def set_motor_speed(self, speeds):
        """speeds: list of [motor_id, speed], motor_id 1-based, speed -1.0~1.0"""
        data = [0x01, len(speeds)]
        for motor_id, speed in speeds:
            data.extend(struct.pack("<Bf", int(motor_id - 1), float(speed)))
        self.send_packet(FUNC_MOTOR, data)

    def odom_timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now

        vx = self.cmd_vx
        vy = self.cmd_vy
        wz = self.cmd_wz

        # Dead-reckoning: rotate body-frame velocity to world frame
        self.pose_x += (vx * math.cos(self.pose_yaw) - vy * math.sin(self.pose_yaw)) * dt
        self.pose_y += (vx * math.sin(self.pose_yaw) + vy * math.cos(self.pose_yaw)) * dt
        self.pose_yaw += wz * dt

        q = yaw_to_quaternion(self.pose_yaw)
        stamp = now.to_msg()

        # Publish odom -> base_link TF
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.pose_x
        t.transform.translation.y = self.pose_y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        # Covariance: low when stopped, higher when moving (dead-reckoning drift)
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            odom.pose.covariance[0] = 1e-9
            odom.pose.covariance[7] = 1e-9
            odom.pose.covariance[35] = 1e-9
        else:
            odom.pose.covariance[0] = 1e-3
            odom.pose.covariance[7] = 1e-3
            odom.pose.covariance[35] = 1e-3
        self.odom_pub.publish(odom)

    def cmd_vel_callback(self, msg):
        # 麦克纳姆轮逆运动学 (官方参数)
        #        x
        # motor1 | ↑ | motor3
        #   +y-  |   |
        # motor2 |   | motor4
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # Capture for odometry dead-reckoning
        self.cmd_vx = vx
        self.cmd_vy = vy
        self.cmd_wz = wz

        wheelbase = 0.1368      # 前后轴距
        track_width = 0.1410    # 左右轴距
        wheel_diameter = 0.065  # 轮径

        vp = wz * (wheelbase + track_width) / 2.0

        m1 = vx - vy - vp
        m2 = vx + vy - vp
        m3 = vx + vy + vp
        m4 = vx - vy + vp

        # 转换为 rps (转/秒)，并按官方取反规则
        def to_rps(v):
            return v / (math.pi * wheel_diameter)

        self.set_motor_speed([
            [1, to_rps(-m1)],
            [2, to_rps(-m2)],
            [3, to_rps(m3)],
            [4, to_rps(m4)],
        ])

    def gimbal_callback(self, msg):
        def angle_to_pulse(angle):
            a = max(0, min(180, angle))
            return int(500 + (a / 180.0) * 2000)

        p_pulse = angle_to_pulse(msg.pitch)
        y_pulse = angle_to_pulse(msg.yaw)

        duration_ms = 100
        # 官方格式: [0x01, dur_lo, dur_hi, count, id, pos_lo, pos_hi, ...]
        data = [0x01, duration_ms & 0xFF, (duration_ms >> 8) & 0xFF, 2]
        data.extend(struct.pack("<BH", 1, p_pulse))
        data.extend(struct.pack("<BH", 2, y_pulse))
        self.send_packet(FUNC_PWM_SERVO, data)

def main(args=None):
    rclpy.init(args=args)
    node = MentorPiBase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
