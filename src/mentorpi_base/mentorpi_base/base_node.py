import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from mentorpi_msgs.msg import Gimbal, MotorStatus, Buzzer
from tf2_ros import TransformBroadcaster
import serial
import struct
import math
import threading
import time


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

FUNC_SYS = 0
FUNC_BUZZER = 2
FUNC_MOTOR = 3
FUNC_PWM_SERVO = 4
FUNC_IMU = 7

# SYS sub-ids
SYS_BATTERY = 0x04

GRAVITY = 9.80665

# Packet parser states
STATE_START1 = 0
STATE_START2 = 1
STATE_FUNC = 2
STATE_LEN = 3
STATE_DATA = 4
STATE_CRC = 5

class MentorPiBase(Node):
    def __init__(self):
        super().__init__('mentorpi_base')

        self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B21250490-if00')
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
        self.buzzer_sub = self.create_subscription(Buzzer, '/buzzer', self.buzzer_callback, 10)

        # Odometry state (command-based dead-reckoning)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        # Ramped (estimated actual) velocity: cmd_vel is a step, the chassis is
        # not — integrating the step over-counts distance at every start/stop.
        self.est_vx = 0.0
        self.est_vy = 0.0
        self.est_wz = 0.0
        self.last_odom_time = self.get_clock().now()

        # Max chassis accel used for the odom ramp model (0 = integrate raw cmd)
        self.declare_parameter('accel_limit_linear', 1.5)    # m/s^2
        self.declare_parameter('accel_limit_angular', 10.0)  # rad/s^2
        self.accel_limit_linear = self.get_parameter('accel_limit_linear').value
        self.accel_limit_angular = self.get_parameter('accel_limit_angular').value

        # 轮径: 标称 0.065, 卷尺标定 2026-07-05 (3x 1m 直线, odom 均值
        # 0.9935 vs 实测 0.98) 得尺度系数 0.9864 -> 0.0641。
        self.declare_parameter('wheel_diameter', 0.0641)
        self.wheel_diameter = self.get_parameter('wheel_diameter').value

        self.declare_parameter('publish_odom_tf', False)
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value

        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.last_cmd_vel_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.02, self.odom_timer_callback)  # 50 Hz
        self.create_timer(0.1, self.watchdog_callback)     # 10 Hz cmd_vel timeout check

        # Start serial receive thread for IMU data
        if self.ser:
            time.sleep(0.5)  # let STM32 settle after port open
            self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
            self._recv_thread.start()
            self.get_logger().info("IMU receive thread started")

    def _recv_loop(self):
        """Background thread: parse incoming packets from STM32."""
        state = STATE_START1
        frame = []
        recv_count = 0

        while rclpy.ok():
            ser = self.ser  # snapshot — watchdog 可能并发把它清成 None
            if ser is None:
                time.sleep(0.2)
                state = STATE_START1
                frame = []
                continue
            try:
                raw = ser.read(64)
            except (OSError, serial.SerialException, AttributeError, TypeError):
                # 让 watchdog 接管重连; 这里只复位解析状态。
                # TypeError: watchdog 并发 close() 会把 pyserial 的 fd 置
                # None, os.read(None) 抛 TypeError —— USB 过流掉线时实测
                # 命中过,不接住的话本线程死掉,重连后 IMU 永久失联。
                time.sleep(0.2)
                state = STATE_START1
                frame = []
                continue
            if not raw:
                continue
            for dat in raw:
                if state == STATE_START1:
                    if dat == 0xAA:
                        state = STATE_START2
                elif state == STATE_START2:
                    if dat == 0x55:
                        state = STATE_FUNC
                    else:
                        state = STATE_START1
                elif state == STATE_FUNC:
                    if dat < 12:  # valid function codes 0-11
                        frame = [dat, 0]
                        state = STATE_LEN
                    else:
                        state = STATE_START1
                elif state == STATE_LEN:
                    frame[1] = dat
                    recv_count = 0
                    state = STATE_DATA if dat > 0 else STATE_CRC
                elif state == STATE_DATA:
                    frame.append(dat)
                    recv_count += 1
                    if recv_count >= frame[1]:
                        state = STATE_CRC
                elif state == STATE_CRC:
                    if checksum_crc8(bytes(frame)) == dat:
                        func = frame[0]
                        data = bytes(frame[2:])
                        try:
                            if func == FUNC_IMU and len(data) == 24:
                                self._publish_imu(data)
                            elif func == FUNC_SYS and len(data) == 3 and data[0] == SYS_BATTERY:
                                self._publish_battery(data[1:])
                        except Exception as e:
                            # rclpy context 关闭时 publish 抛 RCLError;其余
                            # 异常也不能杀本线程(它没有替补)。
                            if not rclpy.ok():
                                return
                            self.get_logger().warn(
                                f"publish failed in recv thread: {e}",
                                throttle_duration_sec=5.0)
                    state = STATE_START1

    def _publish_imu(self, data):
        ax, ay, az, gx, gy, gz = struct.unpack('<6f', data)

        msg = Imu()
        msg.header.frame_id = 'imu_link'
        msg.header.stamp = self.get_clock().now().to_msg()

        # No orientation estimate from raw IMU
        msg.orientation.w = 0.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation_covariance = [
            -1.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
        ]

        # Accelerometer: g -> m/s²
        msg.linear_acceleration.x = ax * GRAVITY
        msg.linear_acceleration.y = ay * GRAVITY
        msg.linear_acceleration.z = az * GRAVITY
        msg.linear_acceleration_covariance = [
            0.0004, 0.0, 0.0,
            0.0, 0.0004, 0.0,
            0.0, 0.0, 0.004,
        ]

        # Gyroscope: deg/s -> rad/s
        msg.angular_velocity.x = math.radians(gx)
        msg.angular_velocity.y = math.radians(gy)
        msg.angular_velocity.z = math.radians(gz)
        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]

        self.imu_pub.publish(msg)

    def _publish_battery(self, data):
        # STM32 SYS packet sub-id 0x04: uint16 LE millivolts of input rail.
        # 反映 STM32 电源输入电压(电池或 DC 适配器),不是 Pi 自己的供电。
        mv = struct.unpack('<H', data)[0]
        msg = BatteryState()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = mv / 1000.0
        msg.current = float('nan')
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = float('nan')
        msg.percentage = float('nan')
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = mv > 1000  # <1V 视为没接电池/采样异常
        self.battery_pub.publish(msg)

    def send_packet(self, func, data):
        if not self.ser:
            return
        buf = [0xAA, 0x55, int(func)]
        buf.append(len(data))
        buf.extend(data)
        buf.append(checksum_crc8(bytes(buf[2:])))
        try:
            self.ser.write(bytes(buf))
        except (OSError, serial.SerialException) as e:
            # 保住进程不被串口瞬态错误(USB线松动 / STM32 reset)杀掉
            # 否则 watchdog 也会跟着死, STM32 上最后一笔速度无人撤销 -> 跑飞
            self.get_logger().error(f"Serial write failed: {e} — closing port, will retry")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def buzzer_callback(self, msg):
        # uint16 LE: freq, on_time_ms, off_time_ms, repeat
        on_ms = max(0, min(0xFFFF, int(msg.on_time * 1000)))
        off_ms = max(0, min(0xFFFF, int(msg.off_time * 1000)))
        data = struct.pack('<HHHH', int(msg.freq), on_ms, off_ms, int(msg.repeat))
        self.send_packet(FUNC_BUZZER, data)

    def set_motor_speed(self, speeds):
        """speeds: list of [motor_id, speed], motor_id 1-based, speed -1.0~1.0"""
        data = [0x01, len(speeds)]
        for motor_id, speed in speeds:
            data.extend(struct.pack("<Bf", int(motor_id - 1), float(speed)))
        self.send_packet(FUNC_MOTOR, data)

    @staticmethod
    def _ramp(current, target, max_delta):
        if max_delta <= 0.0:
            return target
        delta = target - current
        if delta > max_delta:
            delta = max_delta
        elif delta < -max_delta:
            delta = -max_delta
        return current + delta

    def odom_timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        # Guard against scheduler hiccups blowing up a single integration step
        dt = min(dt, 0.1)

        # First-order chassis response: slew estimated velocity toward the
        # commanded one instead of assuming the step is reached instantly.
        self.est_vx = self._ramp(self.est_vx, self.cmd_vx, self.accel_limit_linear * dt)
        self.est_vy = self._ramp(self.est_vy, self.cmd_vy, self.accel_limit_linear * dt)
        self.est_wz = self._ramp(self.est_wz, self.cmd_wz, self.accel_limit_angular * dt)

        vx = self.est_vx
        vy = self.est_vy
        wz = self.est_wz

        # Dead-reckoning: rotate body-frame velocity to world frame
        self.pose_x += (vx * math.cos(self.pose_yaw) - vy * math.sin(self.pose_yaw)) * dt
        self.pose_y += (vx * math.sin(self.pose_yaw) + vy * math.cos(self.pose_yaw)) * dt
        self.pose_yaw += wz * dt

        q = yaw_to_quaternion(self.pose_yaw)
        stamp = now.to_msg()

        # Publish odom -> base_link TF (only when EKF is not handling it)
        if self.publish_odom_tf:
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
        # Covariance: low when stopped, higher when moving (dead-reckoning drift).
        # The EKF fuses the TWIST (vx/vy/vyaw), so twist covariance is the one
        # that matters — leaving it at 0 makes robot_localization treat the
        # commanded velocity as ground truth. There is no encoder feedback, so
        # while moving these are honest "open-loop mecanum" numbers: strafe
        # (vy) slips more than forward (vx), in-place rotation (vyaw) slips
        # worst; the gyro should dominate yaw in the EKF.
        moving = not (vx == 0.0 and vy == 0.0 and wz == 0.0
                      and self.cmd_vx == 0.0 and self.cmd_vy == 0.0 and self.cmd_wz == 0.0)
        if moving:
            pose_var, var_vx, var_vy, var_wz = 1e-3, 0.02, 0.05, 0.2
        else:
            pose_var, var_vx, var_vy, var_wz = 1e-9, 1e-6, 1e-6, 1e-6
        odom.pose.covariance[0] = pose_var
        odom.pose.covariance[7] = pose_var
        odom.pose.covariance[35] = pose_var
        odom.twist.covariance[0] = var_vx    # vx
        odom.twist.covariance[7] = var_vy    # vy
        odom.twist.covariance[35] = var_wz   # vyaw
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
        self.last_cmd_vel_time = self.get_clock().now()

        wheelbase = 0.1368      # 前后轴距
        track_width = 0.1410    # 左右轴距
        wheel_diameter = self.wheel_diameter  # 轮径(标定参数, 见 __init__)

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

    def watchdog_callback(self):
        # 串口断了就尝试重连, 重连后立即发停车
        if self.ser is None:
            try:
                port = self.get_parameter('port').value
                baud = self.get_parameter('baudrate').value
                self.ser = serial.Serial(None, baud, timeout=0.1)
                self.ser.rts = False
                self.ser.dtr = False
                self.ser.setPort(port)
                self.ser.open()
                self.get_logger().info(f"Serial reconnected: {port}")
                # 重连成功必须立即停车 — STM32 上之前可能还在跑
                self.cmd_vx = 0.0
                self.cmd_vy = 0.0
                self.cmd_wz = 0.0
                self.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
            except Exception:
                self.ser = None  # 还连不上, 下次再试
            return

        # 如果 /cmd_vel 超时未到达, 主动停车并清里程计速度
        # 防止 teleop 崩溃 / 手柄断连 时 STM32 上最后一笔速度无人撤销
        elapsed = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        if elapsed > self.cmd_vel_timeout:
            if self.cmd_vx != 0.0 or self.cmd_vy != 0.0 or self.cmd_wz != 0.0:
                self.get_logger().warn(
                    f"/cmd_vel timeout ({elapsed:.2f}s > {self.cmd_vel_timeout}s) — stopping motors")
                self.cmd_vx = 0.0
                self.cmd_vy = 0.0
                self.cmd_wz = 0.0
                self.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前停车,避免 STM32 上残留最后一笔速度继续跑
        try:
            node.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
