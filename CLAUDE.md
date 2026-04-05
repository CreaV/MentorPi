# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

A ROS 2 Jazzy workspace for the MentorPi robot — a Raspberry Pi 5 + RRCLite STM32 mecanum-wheel car with a 2-DOF camera gimbal and MS200 lidar. The workspace lives at `/home/pi/workdir/mentorpi/mentorpi_ws/`.

## Build & Run

```bash
# Build everything (run from mentorpi_ws/)
source /opt/ros/jazzy/setup.bash
colcon build

# Build specific package
colcon build --packages-select mentorpi_base

# Source after build
source install/setup.bash

# Launch full system
ros2 launch mentorpi_bringup mentorpi.launch.py

# Launch with SLAM mapping
ros2 launch mentorpi_bringup mapping.launch.py
```

**Important:** Must use system Python 3.12 (`/usr/bin/python3.12`) for ROS2 Python scripts, not conda's Python 3.13. The `ros2` CLI and `colcon` handle this automatically, but direct `python3 some_script.py` will fail.

There is a duplicate `oradar_lidar` package in the parent `src/p2117_ros/oradar_ros/`. If colcon reports "Duplicate package names", run from `mentorpi_ws/` directory (not the parent).

## Architecture

```
joy_node ─→ /joy ─→ mentorpi_teleop ─→ /cmd_vel ──→ mentorpi_base ─→ Serial ─→ RRCLite STM32
                                     ─→ /gimbal/cmd ─┘               │          ├─ 4x mecanum motors
                                                                      │          └─ 2x gimbal servos
                                                                      ├─→ /odom (Odometry, 50Hz dead-reckoning)
                                                                      └─→ TF: odom → base_link

oradar_scan ─→ /scan (LaserScan, Best Effort QoS)
mentorpi_vision ─→ /camera/image_raw
slam_toolbox ─→ /map (OccupancyGrid) + TF: map → odom
```

Key data flow: Joystick axes → teleop maps to Twist (mecanum kinematics in base_node) and Gimbal → base_node packs binary serial frames with CRC8 lookup table → RRCLite controller.

### TF Tree

```
map → odom → base_link → laser_frame
(slam)  (base_node)   (static, z=0.18m)
```

### Odometry

`base_node` publishes `/odom` at 50Hz via command-based dead-reckoning (integrates `/cmd_vel`). No encoder feedback from STM32 — same approach as vendor SDK. Covariance is set low when stopped, higher when moving, so slam_toolbox relies more on scan matching.

## Packages

| Package | Type | Node | Key File |
|---------|------|------|----------|
| `mentorpi_msgs` | C++ (ament_cmake) | — | `msg/Gimbal.msg`, `msg/MotorStatus.msg` |
| `mentorpi_base` | Python | `base_node` | `base_node.py` — serial protocol, mecanum kinematics, odometry |
| `mentorpi_teleop` | Python | `teleop_node` | `teleop_node.py` — joystick mapping |
| `mentorpi_vision` | Python | `vision_node` | `vision_node.py` — camera capture |
| `mentorpi_bringup` | Python | — | `launch/mentorpi.launch.py`, `mapping.launch.py`, `localization.launch.py` |
| `oradar_lidar` | C++ (ament_cmake) | `oradar_scan` | `src/oradar_scan_node.cpp` — lidar driver |

## Hardware Serial Protocol

Full protocol documented in `docs/hardware_protocol.md`. Critical details:

- **Device:** `/dev/ttyACM0` @ 1,000,000 baud. Must set `rts=False, dtr=False` before opening.
- **Packet:** `[0xAA] [0x55] [Function] [Length] [Data...] [CRC8]`. CRC8 uses lookup table (not bit-by-bit), computed over Function+Length+Data.
- **Motor IDs are 0-indexed** in the protocol (motor 1 sends as 0). Speed is float32 LE in rps.
- **Motors 1,2 are sign-inverted** in the mecanum kinematics (official SDK convention).
- **Servo IDs are 1-based.** PWM position is uint16 LE (500-2500 μs range).
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

## SLAM Mapping & Localization

- **Package:** `slam_toolbox` (system package, install with `sudo apt install ros-jazzy-slam-toolbox`)
- **Map topic:** `/map` (nav_msgs/OccupancyGrid), resolution 0.05m, max range 12.0m
- **Dependency:** Lidar `/scan` (Best Effort QoS) + odometry from `base_node` (dead-reckoning)

### Mapping (建图)

```bash
ros2 launch mentorpi_bringup mapping.launch.py
# 遥控机器人走一圈，建完后保存：
mkdir -p ~/maps
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/pi/maps/my_room'}"
```

- **Config:** `src/mentorpi_bringup/config/slam_toolbox_params.yaml`
- **Node:** `async_slam_toolbox_node` (online async SLAM)
- Saves `.posegraph` + `.data` files

### Localization (定位)

```bash
# 加载默认地图 (/home/pi/maps/my_room)
ros2 launch mentorpi_bringup localization.launch.py

# 指定其他地图
ros2 launch mentorpi_bringup localization.launch.py map_file:=/home/pi/maps/kitchen
```

- **Config:** `src/mentorpi_bringup/config/slam_toolbox_localization_params.yaml`
- **Node:** `localization_slam_toolbox_node`
- `map_start_at_dock: true` — 从保存地图时的位姿启动

## Reference Code

Official vendor SDK is at `/home/pi/workdir/mentorpi/src/` — use it as reference for protocol details but don't modify it. Key files:
- `src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py` — authoritative serial protocol implementation
- `src/driver/controller/controller/mecanum.py` — official mecanum kinematics
- `src/peripherals/peripherals/joystick_control.py` — official joystick mapping
- `src/p2117_ros/oradar_ros/` — lidar driver source (copied to our `src/oradar_lidar/`)
