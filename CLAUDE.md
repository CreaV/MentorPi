# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

A ROS 2 Jazzy workspace for the MentorPi robot — a Raspberry Pi 5 + RRCLite STM32 mecanum-wheel car with a 2-DOF camera gimbal, MS200 lidar, and Orbbec Gemini 2L depth camera. The workspace lives at `/home/pi/workdir/mentorpi/mentorpi_ws/`.

## Build & Run

```bash
# Build everything (run from mentorpi_ws/)
source /opt/ros/jazzy/setup.bash
colcon build

# Build specific package
colcon build --packages-select mentorpi_base

# Source after build
source install/setup.bash

# Launch full system (2D SLAM, with IMU+EKF fusion)
ros2 launch mentorpi_bringup mapping.launch.py

# Launch 3D SLAM (RTAB-Map + Gemini 2L)
ros2 launch mentorpi_bringup rtabmap_mapping.launch.py

# Launch localization with existing 2D map
ros2 launch mentorpi_bringup localization.launch.py
```

**Important:** Must use system Python 3.12 (`/usr/bin/python3.12`) for ROS2 Python scripts, not conda's Python 3.13. The `ros2` CLI and `colcon` handle this automatically, but direct `python3 some_script.py` will fail.

There is a duplicate `oradar_lidar` package in the parent `src/p2117_ros/oradar_ros/`. If colcon reports "Duplicate package names", run from `mentorpi_ws/` directory (not the parent).

## System Architecture

The system has two SLAM modes: **2D** (slam_toolbox + lidar) and **3D** (RTAB-Map + depth camera). Both share the same hardware base.

### Mode 1: 2D SLAM (mapping.launch.py)

```
                        ┌─────────────────────────────────────────────┐
                        │           mentorpi.launch.py                │
                        │                                             │
joy_node → /joy → teleop_node → /cmd_vel ──→ base_node ─→ Serial ──→ RRCLite STM32
                               → /gimbal/cmd ─┘    │                  ├─ 4x mecanum motors
                                                    │                  └─ 2x gimbal servos
                                                    │
                                                    ├─→ /odom (50Hz, dead-reckoning)
                                                    └─→ /imu/data_raw (STM32 IMU)
                                                             │
                                                    madgwick ↓
                                                        /imu/data (with orientation)
                                                             │
                                              /odom ───→ EKF ←─── /imu/data
                                                          │
                                                          ├─→ TF: odom → base_link
                                                          └─→ /odometry/filtered
                        │                                             │
                        └─────────────────────────────────────────────┘

oradar_scan ─→ /scan (LaserScan) ──→ slam_toolbox ─→ /map + TF: map → odom
```

### Mode 2: 3D SLAM (rtabmap_mapping.launch.py) — 异构架构

**前端**(高频 dead-reckoning,跟 2D 模式共用同一套)：

```
base_node ─→ /odom (50Hz, cmd_vel 积分) ─────┐
          ─→ /imu/data_raw (STM32 IMU)       │
                       │                      │
                  madgwick ↓                  │
                  /imu/data (含 orientation)  │
                       │                      ↓
                       └────────────→ EKF ←──/odom
                                       │
                                       ├─→ TF: odom → base_link (50Hz)
                                       └─→ /odometry/filtered
```

**后端**(低频建图 + 周期性 loop closure)：

```
Gemini 2L ─→ /camera/color/image_raw ────┐
           → /camera/depth/image_raw ─────┤→ rtabmap (2Hz) ──→ TF: map → odom
           → /camera/color/camera_info ───┤    ↑               → /cloud_map (累积点云)
                                          │    │               → /map (2D 栅格)
                                          │    │               → /mapData, /mapGraph
                                          │    └── TF lookup (odom→base_link by EKF)
                                          │
oradar_scan ─→ /scan (避障用，不参与 3D 建图)
```

**关键差别(对比旧的纯视觉里程计设计)：**
- **没有 `rgbd_odometry`**：机器人位姿来自 EKF 融合的轮速 + STM32 IMU,~20ms 延迟,45Hz
- **没有相机 IMU**：`enable_accel/gyro: false`,Gemini 2L IMU 不参与
- **rtabmap 通过 TF 拿位姿**：`subscribe_odom_info: false`,自动从 TF 链 `odom → base_link` 读
- **轮速里程计漂移由 loop closure 校正**：rtabmap 在 `map → odom` 上发出修正

收益:机器人 pose 延迟从 ~330ms → ~20ms (16x↓),sync 警告消除,视觉里程计永不 fail,CPU 节省约 20pp。

### TF Trees

**2D SLAM:**
```
map → odom → base_link → laser_frame
(slam_toolbox) (EKF)     (static, z=0.18m)
```

**3D SLAM(异构架构):**
```
map → odom → base_link → camera_link → camera_*_optical_frame
(rtabmap) (EKF)        (static)        (orbbec driver)
                       → laser_frame
                         (static, z=0.18m)
```

**相机-底盘标定**:`base_link → camera_link` 静态 TF 在 `rtabmap_mapping.launch.py` 里手填(默认 x=0.05, z=0.15, 无旋转,**只是估计值,需用尺子量后校准**)。如果相机有俯仰角(如低头看地面),记得填 RPY 的 pitch(弧度制)。

## Packages

| Package | Type | Node(s) | Description |
|---------|------|---------|-------------|
| `mentorpi_msgs` | C++ (ament_cmake) | — | `msg/Gimbal.msg`, `msg/MotorStatus.msg` |
| `mentorpi_base` | Python | `base_node` | Serial protocol, mecanum kinematics, odometry, IMU |
| `mentorpi_teleop` | Python | `teleop_node` | Joystick mapping |
| `mentorpi_vision` | Python | `vision_node` | OpenCV camera capture |
| `mentorpi_bringup` | Python | — | Launch files and config (see below) |
| `oradar_lidar` | C++ (ament_cmake) | `oradar_scan` | MS200 lidar driver |

### External Packages (apt)

| Package | Node | Used in |
|---------|------|---------|
| `slam_toolbox` | `async_slam_toolbox_node` / `localization_slam_toolbox_node` | 2D SLAM |
| `rtabmap_ros` | `rtabmap`, `point_cloud_xyzrgb` | 3D SLAM |
| `imu_filter_madgwick` | `imu_filter_madgwick_node` | Both modes |
| `robot_localization` | `ekf_node` | Both modes |
| `orbbec_camera` | `camera` (Gemini 2L driver) | 3D SLAM |

### Launch Files

| File | Description |
|------|-------------|
| `mentorpi.launch.py` | Base hardware: base_node + STM32 IMU Madgwick + EKF + camera + joy + teleop + lidar |
| `mapping.launch.py` | = mentorpi.launch.py + slam_toolbox (2D mapping) |
| `localization.launch.py` | = mentorpi.launch.py + slam_toolbox localization mode |
| `rtabmap_mapping.launch.py` | Standalone: base_node + STM32 IMU Madgwick + EKF + Gemini 2L (RGB-D only) + RTAB-Map (3D SLAM, 异构架构) |

### Config Files (`src/mentorpi_bringup/config/`)

| File | Description |
|------|-------------|
| `imu_filter.yaml` | Madgwick filter for STM32 IMU (both 2D and 3D modes) |
| `ekf.yaml` | robot_localization EKF: fuses /odom + /imu/data (both 2D and 3D modes) |
| `slam_toolbox_params.yaml` | slam_toolbox mapping config |
| `slam_toolbox_localization_params.yaml` | slam_toolbox localization config |

## base_node — Serial Driver & Sensor Hub

`base_node` handles bidirectional serial communication with the RRCLite STM32:

**Sending (main thread):**
- Motor speed commands (Function=3) from `/cmd_vel`
- Gimbal servo commands (Function=4) from `/gimbal/cmd`

**Receiving (background thread):**
- IMU data (Function=7) → publishes `/imu/data_raw` (sensor_msgs/Imu)
- State machine packet parser identical to official SDK's `recv_task`

**Parameters:**
| Param | Default | Description |
|-------|---------|-------------|
| `port` | `/dev/ttyACM0` | Serial device |
| `baudrate` | `1000000` | Baud rate |
| `publish_odom_tf` | `False` | Publish odom→base_link TF (disable when EKF handles it) |

**Published topics:**
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/odom` | Odometry | 50Hz | Dead-reckoning from cmd_vel integration |
| `/imu/data_raw` | Imu | ~50Hz | Raw accel (m/s²) + gyro (rad/s), no orientation |

## IMU Data Flow

两种模式都使用 STM32 IMU 经过 Madgwick + EKF 融合,Gemini 2L 自带 IMU 不参与 SLAM。

```
STM32 (Function=7, 6×float32: ax,ay,az,gx,gy,gz)
  → base_node recv thread (parse, convert units)
  → /imu/data_raw (accel: g→m/s², gyro: deg/s→rad/s, no orientation)
  → imu_filter_madgwick
  → /imu/data (with orientation quaternion)
  → EKF (fuses with /odom → /odometry/filtered + TF odom→base_link)
```

3D 模式下 Gemini 2L 的 IMU 流通过 launch 参数关掉(`enable_accel/gyro: false`),节省 USB 带宽和 CPU。

## Hardware Serial Protocol

Full protocol documented in `docs/hardware_protocol.md`. Critical details:

- **Device:** `/dev/ttyACM0` @ 1,000,000 baud. Must set `rts=False, dtr=False` before opening.
- **Packet:** `[0xAA] [0x55] [Function] [Length] [Data...] [CRC8]`. CRC8 uses lookup table (not bit-by-bit), computed over Function+Length+Data.
- **Motor IDs are 0-indexed** in the protocol (motor 1 sends as 0). Speed is float32 LE in rps.
- **Motors 1,2 are sign-inverted** in the mecanum kinematics (official SDK convention).
- **Servo IDs are 1-based.** PWM position is uint16 LE (500-2500 μs range).
- **IMU (Function=7):** STM32 auto-reports. 24 bytes = 6×float32 LE (ax,ay,az in g; gx,gy,gz in deg/s).
- **Mecanum parameters:** wheelbase=0.1368m, track_width=0.1410m, wheel_diameter=0.065m.

## Joystick Mapping (Beitong BTP-KP20D)

```
axes[0]=lx  axes[1]=ly  axes[2]=rx  axes[3]=ry  axes[4]=r2  axes[5]=l2  axes[6/7]=dpad
```

- Left stick: ly→linear.x (forward/back), lx→linear.y (strafe)
- Right stick X: angular.z (rotation) — OR gimbal yaw when RB held
- Right stick Y: gimbal pitch when RB held
- RB = buttons[7]. Releasing RB auto-centers gimbal to (90°, 90°).
- Deadzone: 0.1

## Lidar (oradar_lidar)

- **Device:** `/dev/ttyUSB0` @ 230400 baud
- **QoS:** SensorDataQoS (Best Effort) — RViz2 must match
- **Node name hardcoded** as `oradar_ros` in C++ source; launch `name` must match this
- **CMakeLists.txt:** `COMPILE_METHOD` must be `COLCON` (default is `CATKIN`)
- `ctrl+c` may not cleanly stop the node; use `pkill -9 -f oradar_scan`
- Visualization: `view_scan.py` generates `/tmp/scan_view.png` (use `/usr/bin/python3.12`)

## 2D SLAM Mapping & Localization (slam_toolbox)

- **Package:** `slam_toolbox` (install: `sudo apt install ros-jazzy-slam-toolbox`)
- **Odometry source:** EKF-fused (`/odometry/filtered`) via odom→base_link TF
- **Map topic:** `/map` (OccupancyGrid), resolution 0.05m

### Mapping (建图)

```bash
ros2 launch mentorpi_bringup mapping.launch.py
# 遥控机器人走一圈，建完后保存：
mkdir -p ~/maps
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/pi/maps/my_room'}"
```

### Localization (定位)

```bash
ros2 launch mentorpi_bringup localization.launch.py
ros2 launch mentorpi_bringup localization.launch.py map_file:=/home/pi/maps/kitchen
```

## 3D SLAM (RTAB-Map + Gemini 2L)

- **Packages:** `rtabmap_ros`, `orbbec_camera`, `robot_localization` (install: `sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-robot-localization`)
- **架构:** 异构(heterogeneous) — 前端 EKF (轮速+IMU) 出 odom,后端 rtabmap 仅做 mapping + loop closure
- **Odometry source:** EKF (`/odometry/filtered` @ 45Hz, ~20ms 延迟)
- **不再使用 rgbd_odometry**:rtabmap 通过 TF 链 `odom→base_link` 拿位姿(`subscribe_odom_info: false`)

### Mapping (建图)

```bash
mkdir -p ~/rtabmap_maps
ros2 launch mentorpi_bringup rtabmap_mapping.launch.py
# 遥控机器人走一圈，地图自动保存到 ~/rtabmap_maps/rtabmap.db
```

**注意:** 切换架构或频繁中断后 `rtabmap.db` 可能积累 word reference 错位(`addWordRef() Not found word`)。出现这类报错时备份并重建:
```bash
mv ~/rtabmap_maps/rtabmap.db ~/rtabmap_maps/rtabmap.db.bak
```

### Performance Tuning (Pi 5 实测)

经过下面这些优化,新架构在 Pi 5 上可达:**机器人 pose 延迟 ~20ms / 45Hz**,rtabmap 建图 2Hz,核心 CPU 占用约 75%(4 核共 400%)。

`launch_arguments` 给 gemini2L launch:

| Param | Value | Notes |
|-------|-------|-------|
| `color_width/height/fps` | `640/480/15` | 30→15 fps 大幅降低 USB+driver CPU,延迟变化不大 |
| `depth_width/height/fps` | `640/400/15` | 同上 |
| `color_format` | `MJPG` | YUYV 在 Pi 5 上无收益(SDK 转换跟解 MJPG 同等开销) |
| `enable_accel/gyro` | `false` | 异构架构不用相机 IMU |
| `enable_sync_output_accel_gyro` | `false` | 同上 |
| `enable_colored_point_cloud` | `false` | driver 端的彩色点云生成,改用独立 `point_cloud_xyzrgb` 节点 |
| `depth_registration` | `true` | HW D2C 对齐(`align_mode=HW`),ASIC 内做,host CPU 不参与 |

`point_cloud_xyzrgb` 节点参数:

| Param | Value | Notes |
|-------|-------|-------|
| `decimation` | `8` | 降采样,RViz 可视化点云密度足够 |
| `voxel_size` | `0.10` | 同上 |

rtabmap 节点参数:

| Param | Value | Notes |
|-------|-------|-------|
| `subscribe_odom_info` | `false` | 没有 rgbd_odometry,通过 TF 拿 odom |
| `topic_queue_size` / `sync_queue_size` | `20` | 输入 15Hz vs 检测 2Hz,需要队列消化 |
| `Rtabmap/DetectionRate` | `2.0` | Hz, Pi 5 friendly |
| `Kp/MaxFeatures` | `300` | loop closure 用 |
| `RGBD/OptimizeMaxError` | `3.0` | |

**Environment requirements:** RTAB-Map loop closure 仍依赖视觉特征。空白墙、暗光、低纹理环境下 loop closure 检测失败,机器人位姿改靠纯 dead-reckoning(轮速+IMU),漂移会累积直到下次回到有特征的区域触发 loop closure 校正。

**Avoid `Rtabmap/CreateIntermediateNodes=true`** — triggers `Memory.cpp:3473::addLink() Condition (fromS->getWeight() >= 0 && toS->getWeight() >=0) not met` FATAL crash on rtabmap startup. Leave it at the default (false).

**Gemini 2L USB requirements:**
- Must use USB 3.0 port (blue, 5000M) and USB-C 3.0 cable. USB 2.0 (480M) produces `color frame is not decoded` errors and disconnects within 1 second.
- Pi 5 needs `PSU_MAX_CURRENT=5000` in EEPROM (`sudo rpi-eeprom-config --edit`) and `usb_max_current_enable=1` in `/boot/firmware/config.txt` to prevent voltage-drop-induced USB resets when the IR projector kicks in.
- If camera fails to initialize (`uvc_open -6`), try `usbreset 2bc5:0670`. Stale `component_container` processes from a previous launch can also hold the device — check with `ps -ef | grep component_container`.

### RViz2 Visualization

On remote machine:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rviz2
```

- **3D Point Cloud:** Add → PointCloud2 → `/rtabmap/cloud`
- **2D Grid Map:** Add → Map → `/rtabmap/grid_map`
- **Accumulated 3D Map:** Add → MapCloud (rtabmap_rviz_plugins) → `/rtabmap/mapData`
- **IMU Orientation:** Install `ros-jazzy-rviz-imu-plugin`, Add → Imu → `/imu/data`(Madgwick 滤波后)
- **Fixed Frame:** `map`

### Static TF Configuration

| Parent | Child | Translation (m) | Rotation (rad, ZYX) | Notes |
|--------|-------|-----------------|---------------------|-------|
| `base_link` | `camera_link` | x=0.05, z=0.15 | 0 0 0 (yaw pitch roll) | **占位估计值,需用尺子量后改** — 平移误差 → 点云整体偏移;旋转误差 → 点云累积"分层"伪影 |
| `base_link` | `laser_frame` | z=0.18 | 0 0 0 | Lidar mount 高度 |

如果相机有俯仰角(常见,如低头看地面),`base_to_camera` 的 pitch 字段填弧度制(`10° ≈ 0.1745`)。

## Extending the System

### Adding navigation (Nav2)

The system provides all inputs Nav2 needs:
- `/map` or `/rtabmap/grid_map` — costmap source
- `odom → base_link` TF — robot localization (EKF)
- `/scan` — obstacle detection
- `/cmd_vel` — velocity commands

### Switching between 2D and 3D modes

The two modes are independent launch files sharing the same EKF-based frontend (base_node + STM32 IMU madgwick + ekf_node). They should not run simultaneously — both publish `odom → base_link` TF (from EKF) and `map → odom` TF (slam_toolbox vs rtabmap), which would conflict.

主要差别:
- **2D 模式**:lidar `/scan` → slam_toolbox 出 `map → odom`
- **3D 模式**:Gemini 2L RGB-D → rtabmap 出 `map → odom` + 累积彩色点云

## Reference Code

Official vendor SDK is at `/home/pi/workdir/mentorpi/src/` — use it as reference for protocol details but don't modify it. Key files:
- `ros_robot_controller_sdk.py` — authoritative serial protocol (including IMU: `get_imu()`, `packet_report_imu()`)
- `ros_robot_controller_node.py` — official IMU publishing (`pub_imu_data()`)
- `controller/mecanum.py` — official mecanum kinematics
- `peripherals/joystick_control.py` — official joystick mapping
- `p2117_ros/oradar_ros/` — lidar driver source (copied to our `src/oradar_lidar/`)