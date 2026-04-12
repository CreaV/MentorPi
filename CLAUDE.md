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

### Mode 2: 3D SLAM (rtabmap_mapping.launch.py)

```
Gemini 2L ─→ /camera/color/image_raw ───┐
           → /camera/depth/image_raw ────┤→ rgbd_odometry ──→ TF: odom → base_link
           → /camera/gyro_accel/sample   │        ↑
                      │                  │   /camera/imu/data
                 madgwick ───────────────┘   (with orientation)
                      │
                      └─→ /camera/imu/data

base_node ─→ /odom (备用，未接入 EKF)
           → /imu/data_raw (STM32 IMU，3D 模式下未使用)

rgbd_odometry ──→ rtabmap (SLAM) ──→ TF: map → odom
                                   → /rtabmap/grid_map (2D occupancy)
                                   → /rtabmap/cloud (3D point cloud)

oradar_scan ─→ /scan (2D 避障用，不参与 3D 建图)
```

### TF Trees

**2D SLAM:**
```
map → odom → base_link → laser_frame
(slam_toolbox) (EKF)     (static, z=0.18m)
```

**3D SLAM:**
```
map → odom → base_link → camera_link → camera_*_optical_frame
(rtabmap) (rgbd_odom)   (static)      (orbbec driver)
                       → laser_frame
                         (static, z=0.18m)
```

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
| `rtabmap_ros` | `rgbd_odometry`, `rtabmap`, `point_cloud_xyzrgb` | 3D SLAM |
| `imu_filter_madgwick` | `imu_filter_madgwick_node` | Both modes |
| `robot_localization` | `ekf_node` | 2D SLAM |
| `orbbec_camera` | `camera` (Gemini 2L driver) | 3D SLAM |

### Launch Files

| File | Description |
|------|-------------|
| `mentorpi.launch.py` | Base hardware: base_node + STM32 IMU Madgwick + EKF + camera + joy + teleop + lidar |
| `mapping.launch.py` | = mentorpi.launch.py + slam_toolbox (2D mapping) |
| `localization.launch.py` | = mentorpi.launch.py + slam_toolbox localization mode |
| `rtabmap_mapping.launch.py` | Standalone: base_node + Gemini 2L (RGB-D+IMU) + Madgwick + RTAB-Map (3D SLAM) |

### Config Files (`src/mentorpi_bringup/config/`)

| File | Description |
|------|-------------|
| `imu_filter.yaml` | Madgwick filter for STM32 IMU (2D mode) |
| `ekf.yaml` | robot_localization EKF: fuses /odom + /imu/data (2D mode) |
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

### STM32 IMU (2D SLAM mode)

```
STM32 (Function=7, 6×float32: ax,ay,az,gx,gy,gz)
  → base_node recv thread (parse, convert units)
  → /imu/data_raw (accel: g→m/s², gyro: deg/s→rad/s, no orientation)
  → imu_filter_madgwick
  → /imu/data (with orientation quaternion)
  → EKF (fuses with /odom → odom→base_link TF)
```

### Gemini 2L IMU (3D SLAM mode)

```
Gemini 2L hardware IMU
  → /camera/gyro_accel/sample (raw accel + gyro, no orientation)
  → imu_filter_madgwick (camera_imu_filter)
  → /camera/imu/data (with orientation quaternion)
  → rgbd_odometry (visual-inertial odometry)
```

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

- **Packages:** `rtabmap_ros`, `orbbec_camera` (install: `sudo apt install ros-jazzy-rtabmap-ros`)
- **Odometry:** Visual-inertial (rgbd_odometry + Gemini 2L IMU via Madgwick)
- **No EKF / No wheel odometry** — pure visual SLAM

### Mapping (建图)

```bash
mkdir -p ~/rtabmap_maps
ros2 launch mentorpi_bringup rtabmap_mapping.launch.py
# 遥控机器人走一圈，地图自动保存到 ~/rtabmap_maps/rtabmap.db
```

### RTAB-Map Tuning Notes

Key parameters in `rtabmap_mapping.launch.py` (rgbd_odometry node):

| Param | Value | Notes |
|-------|-------|-------|
| `Vis/MinInliers` | 10 | Default 20. Lowered to reduce "Not enough inliers" failures |
| `Vis/MaxFeatures` | 700 | More features = more matching candidates |
| `Vis/InlierDistance` | 0.1 | Relaxed inlier distance threshold |
| `Odom/ResetCountdown` | 5 | Allow 5 consecutive failures before odometry reset |
| `wait_imu_to_init` | true | Wait for IMU gravity alignment before starting |

**Environment requirements:** RTAB-Map needs visual features. Avoid pointing camera at blank walls, uniform surfaces, or very dark areas. Best results with textured environments (furniture, bookshelves, patterned floors).

### RViz2 Visualization

On remote machine:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rviz2
```

- **3D Point Cloud:** Add → PointCloud2 → `/rtabmap/cloud`
- **2D Grid Map:** Add → Map → `/rtabmap/grid_map`
- **Accumulated 3D Map:** Add → MapCloud (rtabmap_rviz_plugins) → `/rtabmap/mapData`
- **IMU Orientation:** Install `ros-jazzy-rviz-imu-plugin`, Add → Imu → `/camera/imu/data`
- **Fixed Frame:** `map`

### Static TF Configuration

| Parent | Child | Translation | Notes |
|--------|-------|-------------|-------|
| `base_link` | `camera_link` | x=0.05, z=0.15 | Camera mount position (adjust to actual) |
| `base_link` | `laser_frame` | z=0.18 | Lidar mount height |

## Extending the System

### Adding wheel odometry to 3D SLAM

If visual odometry fails in low-texture environments, add EKF fusion:

1. Add `imu_filter_madgwick` for STM32 IMU → `/imu/data`
2. Add `ekf_node` fusing `/odom` + `/imu/data` → `/odometry/filtered`
3. Set rgbd_odometry `guess_frame_id: odom` to use wheel odom as initial guess

### Adding navigation (Nav2)

The system provides all inputs Nav2 needs:
- `/map` or `/rtabmap/grid_map` — costmap source
- `odom → base_link` TF — robot localization
- `/scan` — obstacle detection
- `/cmd_vel` — velocity commands

### Switching between 2D and 3D modes

The two modes are independent launch files sharing the same hardware base. They should not run simultaneously (both publish `odom → base_link` TF).

## Reference Code

Official vendor SDK is at `/home/pi/workdir/mentorpi/src/` — use it as reference for protocol details but don't modify it. Key files:
- `ros_robot_controller_sdk.py` — authoritative serial protocol (including IMU: `get_imu()`, `packet_report_imu()`)
- `ros_robot_controller_node.py` — official IMU publishing (`pub_imu_data()`)
- `controller/mecanum.py` — official mecanum kinematics
- `peripherals/joystick_control.py` — official joystick mapping
- `p2117_ros/oradar_ros/` — lidar driver source (copied to our `src/oradar_lidar/`)
