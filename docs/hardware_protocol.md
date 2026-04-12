# MentorPi 硬件通信协议

## 串口配置

| 参数 | 值 |
|------|-----|
| 设备 | `/dev/ttyACM0` |
| 波特率 | 1,000,000 bps |
| RTS | False |
| DTR | False |
| Timeout | 0.1s |

初始化顺序：先设置 rts/dtr 为 False，再 open 端口。

## 数据包格式

```
[0xAA] [0x55] [Function] [Length] [Data...] [CRC8]
```

| 字段 | 长度 | 说明 |
|------|------|------|
| Header | 2 bytes | 固定 `0xAA 0x55` |
| Function | 1 byte | 功能码 |
| Length | 1 byte | Data 段长度 |
| Data | N bytes | 负载数据 |
| CRC8 | 1 byte | 校验，范围覆盖 Function + Length + Data |

## CRC8 校验

使用查表法（非位运算），表见 `base_node.py` 中 `crc8_table`。

计算范围：`buf[2:]`，即 Function、Length、Data 三个字段。

## 功能码

| 码 | 名称 | 说明 |
|----|------|------|
| 0 | SYS | 系统控制 |
| 1 | LED | LED 控制 |
| 2 | BUZZER | 蜂鸣器 |
| 3 | MOTOR | 电机控制 |
| 4 | PWM_SERVO | PWM 舵机（云台） |
| 5 | BUS_SERVO | 总线舵机 |
| 6 | KEY | 按键 |
| 7 | IMU | IMU 数据 |
| 8 | GAMEPAD | 手柄 |
| 9 | SBUS | 航模遥控 |
| 10 | OLED | OLED 显示 |
| 11 | RGB | RGB LED |

---

## 电机控制 (Function=3)

### 数据格式

```
[0x01] [count] [id_0, speed_float] [id_1, speed_float] ...
```

| 字段 | 类型 | 说明 |
|------|------|------|
| SubCmd | uint8 | 固定 `0x01` |
| count | uint8 | 电机数量 |
| id | uint8 | 电机 ID，**0-indexed**（电机1发0，电机2发1...） |
| speed | float32 LE | 速度，单位 rps（转/秒） |

### 电机布局（麦克纳姆轮）

```
        前方 (x+)
motor1  |  ↑  |  motor3
  (y+)← |     | →(y-)
motor2  |     |  motor4
        后方
```

- 电机 1, 2：左侧
- 电机 3, 4：右侧

### 底盘物理参数

| 参数 | 值 | 说明 |
|------|-----|------|
| wheelbase | 0.1368 m | 前后轴距 |
| track_width | 0.1410 m | 左右轴距 |
| wheel_diameter | 0.065 m | 轮径 |

### 麦克纳姆轮逆运动学

```python
vp = angular_z * (wheelbase + track_width) / 2.0

motor1 = linear_x - linear_y - vp
motor2 = linear_x + linear_y - vp
motor3 = linear_x + linear_y + vp
motor4 = linear_x - linear_y + vp

# 转换为 rps，电机1,2取反
speed1 = -motor1 / (pi * wheel_diameter)
speed2 = -motor2 / (pi * wheel_diameter)
speed3 =  motor3 / (pi * wheel_diameter)
speed4 =  motor4 / (pi * wheel_diameter)
```

注意：电机 1, 2 速度**取反**，电机 3, 4 正常。

### 运动示例

| 动作 | linear_x | linear_y | angular_z |
|------|----------|----------|-----------|
| 前进 | +0.3 | 0 | 0 |
| 后退 | -0.3 | 0 | 0 |
| 左平移 | 0 | +0.3 | 0 |
| 右平移 | 0 | -0.3 | 0 |
| 原地左转 | 0 | 0 | +1.0 |
| 原地右转 | 0 | 0 | -1.0 |

### 里程计（Odometry）

`base_node` 以 50Hz 发布 `/odom` 话题，使用指令积分（dead-reckoning）方式：

```python
# 将 body 坐标系速度旋转到 world 坐标系
dx = (vx * cos(yaw) - vy * sin(yaw)) * dt
dy = (vx * sin(yaw) + vy * cos(yaw)) * dt
dyaw = wz * dt
```

- **无编码器反馈**：STM32 协议未实现电机状态回传，官方 SDK 同样使用指令积分
- **漂移补偿**：2D 模式依赖 EKF + slam_toolbox 校正；3D 模式依赖视觉里程计
- **协方差**：静止时设为 1e-9，运动时设为 1e-3
- **odom→base_link TF**：参数 `publish_odom_tf` 控制（默认 False，由 EKF 或 rgbd_odometry 接管）

---

## IMU 数据读取 (Function=7)

### 协议格式

STM32 主动上报 IMU 数据，无需请求命令。

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| ax | float32 LE | g | 加速度 X |
| ay | float32 LE | g | 加速度 Y |
| az | float32 LE | g | 加速度 Z |
| gx | float32 LE | deg/s | 角速度 X |
| gy | float32 LE | deg/s | 角速度 Y |
| gz | float32 LE | deg/s | 角速度 Z |

共 24 字节（6 × float32）。

### 解析示例

```python
ax, ay, az, gx, gy, gz = struct.unpack('<6f', data)  # data = 24 bytes
```

### base_node 中的实现

`base_node` 启动后台接收线程，用状态机解析串口数据包（与官方 SDK `recv_task` 一致）：

1. **接收线程** (`_recv_loop`)：逐字节状态机，匹配 `0xAA 0x55` 头 → Function → Length → Data → CRC8 校验
2. **IMU 发布** (`_publish_imu`)：Function=7 且 data 长度=24 时触发
3. **单位转换**：加速度 g → m/s²（×9.80665），角速度 deg/s → rad/s
4. **orientation**：不估计（`orientation_covariance[0] = -1.0`，ROS 惯例表示无效）
5. **话题**：`/imu/data_raw`（`sensor_msgs/Imu`）

### 注意事项

- STM32 上电后自动上报，无需发送使能命令
- 串口打开后需等待约 0.5 秒让 STM32 稳定
- 接收线程与发送（电机/舵机命令）共用同一串口，Python `serial.Serial` 读写线程安全
- `/dev/ttyACM0` 重启后才可靠出现；如缺失，检查 USB 连接或重启 Pi

---

## PWM 舵机/云台控制 (Function=4)

### 数据格式

```
[0x01] [dur_lo] [dur_hi] [count] [id, pos_lo, pos_hi] ...
```

| 字段 | 类型 | 说明 |
|------|------|------|
| SubCmd | uint8 | 固定 `0x01` |
| duration | uint16 LE | 运动时间（毫秒） |
| count | uint8 | 舵机数量 |
| id | uint8 | 舵机 ID，**1-based** |
| position | uint16 LE | 脉宽值 |

### 舵机分配

| ID | 功能 |
|----|------|
| 1 | Pitch（俯仰） |
| 2 | Yaw（偏航） |

### 角度到脉宽转换

```python
pulse = 500 + (angle / 180.0) * 2000
```

| 角度 | 脉宽 |
|------|------|
| 0° | 500 |
| 45° | 1000 |
| 90° | 1500 (中位) |
| 135° | 2000 |
| 180° | 2500 |

### 示例

云台回中位（pitch=90°, yaw=90°），500ms 运动时间：

```python
set_pwm_servo(ser, 500, [[1, 1500], [2, 1500]])
```

---

## 手柄映射（北通 BTP-KP20D）

### 轴映射

```
axes[0] = lx  左摇杆X（左右）
axes[1] = ly  左摇杆Y（前后）
axes[2] = rx  右摇杆X（左右）
axes[3] = ry  右摇杆Y（上下）
axes[4] = r2  右扳机
axes[5] = l2  左扳机
axes[6] = hat_x  D-pad X
axes[7] = hat_y  D-pad Y
```

### 麦克纳姆底盘操控映射

| 摇杆 | 轴 | 功能 |
|------|-----|------|
| 左摇杆 Y (ly) | axes[1] | linear.x 前后 |
| 左摇杆 X (lx) | axes[0] | linear.y 横移 |
| 右摇杆 X (rx) | axes[2] | angular.z 旋转 |
| RB + 右摇杆 X | axes[2] | 云台 yaw (按住RB时) |
| RB + 右摇杆 Y | axes[3] | 云台 pitch (按住RB时) |

- 死区：0.1（低于此值视为零）
- 按住 RB (buttons[7]) 切换右摇杆为云台控制，松开时云台自动回中

---

## ROS2 话题接口

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/cmd_vel` | geometry_msgs/Twist | → base_node | linear.x 前后, linear.y 横移, angular.z 旋转 |
| `/gimbal/cmd` | mentorpi_msgs/Gimbal | → base_node | pitch/yaw 角度 (0-180°) |
| `/odom` | nav_msgs/Odometry | ← base_node | 里程计（指令积分，50Hz） |
| `/map` | nav_msgs/OccupancyGrid | ← slam_toolbox | SLAM 地图（仅 mapping 模式） |
| `/camera/image_raw` | sensor_msgs/Image | ← vision_node | BGR8 图像 |
| `/joy` | sensor_msgs/Joy | ← joy_node | 手柄原始数据 |
| `/scan` | sensor_msgs/LaserScan | ← oradar_scan | MS200 激光雷达 |

---

## 激光雷达 (MS200)

### 硬件参数

| 参数 | 值 |
|------|-----|
| 型号 | MS200 |
| 串口 | `/dev/ttyUSB0` (CH340 转换器) |
| 波特率 | 230400 |
| 测距范围 | 0.05 ~ 12.0 m |
| 扫描角度 | 0° ~ 360° |
| 扫描频率 | 10 Hz (~450 点/帧) |

### ROS2 驱动

| 项目 | 值 |
|------|-----|
| 驱动包 | `oradar_lidar` (源码: `src/oradar_lidar/`) |
| 可执行文件 | `oradar_scan` |
| 话题 | `/scan` (sensor_msgs/LaserScan) |
| QoS | Best Effort (SensorDataQoS) |
| Frame ID | `laser_frame` |
| TF | `base_link` → `laser_frame` (z=0.18m) |

### 驱动来源与编译

源码来自官方 SDK (`src/p2117_ros/oradar_ros/`)，复制到 `mentorpi_ws/src/oradar_lidar/`。

编译注意事项：
- `CMakeLists.txt` 中 `COMPILE_METHOD` 必须设为 `COLCON`（默认是 CATKIN）
- 驱动节点名硬编码为 `oradar_ros`，launch 中 `name` 必须匹配
- 已修复：空帧 (point_nums=0) 导致 frame_id 丢失的 bug

### 单独启动

```bash
ros2 run oradar_lidar oradar_scan --ros-args \
  -p device_model:=MS200 \
  -p port_name:=/dev/ttyUSB0 \
  -p baudrate:=230400 \
  -p scan_topic:=/scan \
  -p frame_id:=laser_frame
```

### 可视化

**本地生成俯视图（推荐）：**

```bash
/usr/bin/python3.12 view_scan.py
# 图片输出到 /tmp/scan_view.png，持续更新
```

注意：必须用系统 Python 3.12（`/usr/bin/python3.12`），不能用 conda 的 Python 3.13。

**RViz2 远程查看：**

跨机器使用 RViz2 时需注意：
- Fixed Frame 设为 `laser_frame`
- LaserScan 的 Reliability Policy 设为 `Best Effort`
- 如果看不到点云，通常是 DDS 问题，建议两端都使用 Cyclone DDS：
  ```bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```

### 已知问题

- `ctrl+c` 可能无法正常关闭 oradar_scan 节点，需要 `pkill -9 -f oradar_scan`
- FastDDS 跨网络传输 LaserScan 数据不稳定，建议切换 Cyclone DDS

---

## SLAM 建图

### 依赖安装

```bash
sudo apt install ros-jazzy-slam-toolbox
```

### 启动建图

```bash
source install/setup.bash
ros2 launch mentorpi_bringup mapping.launch.py
```

此 launch 文件包含完整机器人启动（底盘、雷达、遥控）+ slam_toolbox 异步建图节点。

### TF 树

```
map → odom → base_link → laser_frame
(slam_toolbox)  (base_node)   (静态, z=0.18m)
```

### 配置文件

`src/mentorpi_bringup/config/slam_toolbox_params.yaml`

关键参数：
- 地图分辨率：0.05m
- 最大激光距离：12.0m
- 地图更新间隔：2.0s（针对 Pi 5 性能优化）
- 回环检测：已启用

### 保存地图

```bash
mkdir -p ~/maps
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/pi/maps/my_room'}"
```

生成文件：
- `my_room.posegraph` — 位姿图
- `my_room.data` — 扫描数据

### 定位（使用已有地图）

```bash
# 加载默认地图 (/home/pi/maps/my_room)
ros2 launch mentorpi_bringup localization.launch.py

# 指定其他地图文件（不含扩展名）
ros2 launch mentorpi_bringup localization.launch.py map_file:=/home/pi/maps/kitchen
```

配置文件：`src/mentorpi_bringup/config/slam_toolbox_localization_params.yaml`

关键参数：
- `mode: localization` — 定位模式，不更新地图
- `map_file_name` — 地图路径（由 launch 参数覆盖）
- `map_start_at_dock: true` — 从保存地图时的位姿启动
- `do_loop_closing: false` — 定位模式关闭回环检测

### 验证

```bash
# 检查里程计
ros2 topic echo /odom --once

# 检查 TF 树完整性
ros2 run tf2_tools view_frames

# 检查地图话题
ros2 topic echo /map --once
```
