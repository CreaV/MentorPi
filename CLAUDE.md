# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

A ROS 2 Jazzy workspace for the MentorPi robot — a Raspberry Pi 5 + RRCLite STM32 mecanum-wheel car with an MS200 lidar, an Orbbec Gemini 2L depth camera, and an optional LeRobot SO-101 arm. The old 2-DOF camera gimbal has been removed; the final camera mount and new extrinsic are still pending. The deployed workspace lives at `/home/pi/workdir/mentorpi/mentorpi_ws/`.

## Continuation & Handoff Protocol

For SO-101 work, start with `docs/so101_handoff.md`. It is the current-state handoff; old integration reports and commit logs are historical evidence, not an execution plan.

At the start of any continuation:

1. Run `git status --short --branch` and classify every dirty file before editing. Preserve user-owned changes; in particular, `src/mentorpi_supervisor/foxglove_layout/loc_check.json` has been user-owned during the SO-101 work and must not be staged or reverted without explicit instruction.
2. Read the current handoff, then the relevant parameterized sources. If the handoff conflicts with source plus reproducible validation, source/validation wins and the handoff must be rewritten in place. Do not accumulate correction banners on top of obsolete plans.
3. Separate facts by authority: user-confirmed physical facts > deterministic CAD/STL measurements > clearly labeled assumptions. Never silently promote an inference to a measurement.
4. Edit sources first: xacro/Python CAD/config generators. Regenerate STEP/STL/URDF/GLB and package copies, then run proportional validation. Do not patch generated artifacts as the source of truth.
5. Calibration and appearance are separate. Never change a TF calibration merely to make a render look centered; use visual origins for mesh-intrinsic offsets and physical experiments for joint transforms.

For mechanical/URDF changes, the minimum handoff record is: current invariants, authoritative files, generated artifacts, exact validation results, remaining physical inputs, next ordered action, blocker/owner, baseline commit, push state, and user-owned dirty files. Replace a stale handoff instead of preserving a chronological transcript.

Important generation traps:

- Top-level xacro uses `$(find mentorpi_description)` and can read the installed package copy. Rebuild/source `mentorpi_description` before final generation, or deliberately generate from the direct source xacro while diagnosing.
- The printable Python CAD file and xacro placement must change together; after regenerating STL, synchronize the package copy under `src/mentorpi_description/meshes/accessories/`.
- CAD inspection must include deterministic geometry facts plus at least one reviewed snapshot.
- The current three.js-family URDF viewers can scatter fixed-joint subtrees. Use `mechanical/urdf/bake_urdf_glb.py` and the baked zero-pose GLB for reliable whole-robot visual review.

## Build & Run

```bash
# Build everything (run from mentorpi_ws/)
source /opt/ros/jazzy/setup.bash
colcon build

# Build specific package
colcon build --packages-select mentorpi_base

# Source after build
source install/setup.bash

# Remote operation entry point (Foxglove Studio client over WebSocket)
# Brings up base + foxglove_bridge + supervisor; modes are switched by
# calling /mode/set service from Foxglove (no relaunch).
ros2 launch mentorpi_supervisor remote.launch.py

# CLI shortcuts for direct full-stack runs (no supervisor):
ros2 launch mentorpi_bringup mapping.launch.py            # = base + slam_2d
ros2 launch mentorpi_bringup rtabmap_mapping.launch.py    # = base + slam_3d
ros2 launch mentorpi_bringup localization.launch.py       # = base + loc_2d
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
(slam_toolbox) (EKF)     (static, xyz=-0.012242 0 0.092501)
```

**3D SLAM(异构架构):**
```
map → odom → base_link → camera_link → camera_*_optical_frame
(rtabmap) (EKF)        (static)        (orbbec driver)
                       → laser_frame
                         (static, xyz=-0.012242 0 0.092501)
```

**固定 TF 的源头是 URDF**：`base_link → imu_link/camera_link/laser_frame` 由 `mentorpi_description/urdf/mecanum.xacro` 定义，`base.launch.py` 的 robot_state_publisher 以 `runtime_mode:=true` 渲染发布。雷达是已恢复的直装变换；旧 2-DOF 云台已拆除，因此当前 `camera_joint` 只作历史记录，最终相机支架固定后必须重新做 AprilTag 标定。轮子是 continuous joint，由 joint_state_publisher 发零位 `/joint_states` 补齐 TF（无编码器）。

**相机生命周期**:Gemini 2L 在 `base.launch.py` 里**常驻**(15fps RGB-D),`/camera/color/image_raw` 始终可订,跨模式切换不重启相机。`slam_3d` 模式只在已运行的相机流上加 rtabmap + 点云;2D / loc / idle 模式也能直接用相机预览(虽然没用到 depth)。永久代价:相机 driver ~15-20% CPU on Pi 5。

## Packages

| Package | Type | Node(s) | Description |
|---------|------|---------|-------------|
| `mentorpi_msgs` | C++ (ament_cmake) | — | `msg/Gimbal.msg`, `msg/MotorStatus.msg`, `msg/Buzzer.msg`, `srv/SetMode.srv`, `srv/ListMaps.srv`, `action/MotionPrimitive.action` |
| `mentorpi_base` | Python | `base_node` | Serial protocol, mecanum kinematics, odometry, IMU |
| `mentorpi_motion` | Python | `motion_node` | 有界运动原语 action server（语音/VLA/agent 的执行底座，base.launch 常驻） |
| `mentorpi_teleop` | Python | `teleop_node` | Joystick mapping |
| `mentorpi_bringup` | Python | — | Launch files and config (see below) |
| `mentorpi_description` | Python | — | URDF/xacro + STL meshes;`runtime_mode` arg 区分真机 TF 模型与离线完整模型 |
| `mentorpi_supervisor` | Python | `supervisor_node` | Mode switcher; subprocess-launches slam_2d/slam_3d/loc_2d on request |
| `oradar_lidar` | C++ (ament_cmake) | `oradar_scan` | MS200 lidar driver |

### External Packages (apt)

| Package | Node | Used in |
|---------|------|---------|
| `slam_toolbox` | `async_slam_toolbox_node` / `localization_slam_toolbox_node` | 2D SLAM |
| `rtabmap_ros` | `rtabmap`, `point_cloud_xyzrgb` | 3D SLAM |
| `imu_filter_madgwick` | `imu_filter_madgwick_node` | Both modes |
| `robot_localization` | `ekf_node` | Both modes |
| `orbbec_camera` | `camera` (Gemini 2L driver) | 3D SLAM |
| `foxglove_bridge` | `foxglove_bridge` | 桌面 Foxglove Studio |
| `rosbridge_suite` | `rosbridge_websocket` | 移动 SPA roslibjs ws 桥 |
| `web_video_server` | `web_video_server` | 移动 SPA MJPEG 视频流 |

### Launch Files

Architecture: **base 常驻 + 模式按需挂载**。`base.launch.py` 在 `remote.launch.py` 启动时拉起，supervisor 负责切换 mode-only 子 launch（slam_2d / slam_3d / loc_2d），切换不重启 base，里程计/TF 不归零。

| File | Package | Description |
|------|---------|-------------|
| `base.launch.py` | mentorpi_bringup | 常驻硬件: base_node + STM32 IMU Madgwick + EKF + camera_watchdog (托管 Gemini 2L) + joy + teleop + lidar + robot_state_publisher (URDF 固定 TF: imu/camera/laser) + joint_state_publisher (轮子零位) |
| `camera.launch.py` | mentorpi_bringup | Gemini 2L (RGB-D 15fps, IMU off)。**不直接进别的 launch** —— 由 `camera_watchdog` 节点 spawn 并监测 `/camera/depth/camera_info`,帧停发 >20s 或进程死亡时自动 `usbreset` + 重启 driver(修 openUsbDevice 卡死) |
| `slam_2d.launch.py` | mentorpi_bringup | 仅 slam_toolbox 异步建图 |
| `slam_3d.launch.py` | mentorpi_bringup | 仅 rtabmap + point_cloud_xyzrgb (相机已在 base 里跑)；融合 lidar `/scan` 做 NeighborLinkRefining + proximity detection 抗轮速打滑 |
| `loc_2d.launch.py` | mentorpi_bringup | 仅 slam_toolbox 定位 (吃 `map_file:=...`) |
| `loc_3d.launch.py` | mentorpi_bringup | rtabmap 定位模式 (只读已有 .db，吃 `database_path:=...`)；重启后恢复 map 系位姿，供 live_rerun.py 3D 场景实时显示 |
| `mapping.launch.py` | mentorpi_bringup | CLI 包装: base + slam_2d |
| `rtabmap_mapping.launch.py` | mentorpi_bringup | CLI 包装: base + slam_3d (吃 `database_path:=...`) |
| `localization.launch.py` | mentorpi_bringup | CLI 包装: base + loc_2d |
| `remote.launch.py` | mentorpi_supervisor | 远程操作入口: base + foxglove_bridge + supervisor_node |

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
- Buzzer commands (Function=2) from `/buzzer` (`mentorpi_msgs/Buzzer`: `freq` Hz, `on_time`/`off_time` seconds, `repeat` cycles). Used by supervisor for the startup chime.

**Receiving (background thread):**
- IMU data (Function=7) → publishes `/imu/data_raw` (sensor_msgs/Imu)
- SYS battery (Function=0, sub-id 0x04) → publishes `/battery` (sensor_msgs/BatteryState; voltage 单位 V,反映 STM32 电源输入电压)
- State machine packet parser identical to official SDK's `recv_task`

**Parameters:**
| Param | Default | Description |
|-------|---------|-------------|
| `port` | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B21250490-if00` | Serial device (by-id 稳定路径,避免 ttyACM 编号在 USB 重枚举后漂移) |
| `baudrate` | `1000000` | Baud rate |
| `publish_odom_tf` | `False` | Publish odom→base_link TF (disable when EKF handles it) |
| `accel_limit_linear` | `1.5` m/s² | odom 积分用的底盘加速度斜坡模型（cmd_vel 是阶跃,底盘不是;0=关闭直接积分指令） |
| `accel_limit_angular` | `10.0` rad/s² | 同上,角加速度 |
| `gyro_bias_estimation` | `True` | 停车时在线估计陀螺仪零偏并在 `/imu/data_raw` 中扣除。实测零偏 gz≈+0.28°/s,不扣会被 EKF 积成 ~15°/min 的 yaw 漂移(定位模式下激光相对地图整体旋转) |
| `obstacle_guard` | `True` | 激光避障兜底:订 `/scan` 算前/后/左/右四扇区(各 90°)最近障碍,"撞向障碍"的平移分量在 `guard_slow_distance`(0.6m)内线性减速、`guard_stop_distance`(0.3m,从雷达中心起算)内清零。只拦危险分量——被挡后仍可倒车/横移/旋转脱困。拦截所有 cmd_vel 来源(手柄/手机/Foxglove/VLA)。阻挡状态发 `/guard/blocked`(std_msgs/String,方向逗号连接,空=放行)。雷达停更 >`guard_scan_timeout`(1s)自动放行。全部参数热更新 |

**里程计协方差**:`/odom` 的 **twist** 协方差才是 EKF 实际消费的(融合 vx/vy)。运动时 vx=0.02、vy=0.05(横移打滑更狠)、vyaw=0.2(原地旋转打滑最狠,EKF 已改为不融合它,yaw 率来自陀螺仪);静止时 1e-6 锁死漂移。

**Published topics:**
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/odom` | Odometry | 50Hz | Dead-reckoning from cmd_vel integration |
| `/imu/data_raw` | Imu | ~50Hz | Raw accel (m/s²) + gyro (rad/s), no orientation |
| `/battery` | BatteryState | ~1Hz | STM32 电源输入电压(V),Foxglove Gauge 量程 6.4–8.4V(2S lipo 真实区间:8.4=满电,6.6 该停机充电,≈4.4V=电源开关没开的寄生电压,指针触底+`present` 仍 true);`present=false` 表示采样异常或未接电池 |

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

**EKF 融合配置**(`ekf.yaml`):odom0 (/odom) 只融合 **vx/vy**;imu0 (/imu/data) 只融合 **vyaw**(陀螺仪 z 轴角速率,直接、低延迟、不受打滑影响)。此前 odom0 的 vyaw(指令角速率,麦轮原地旋转打滑最严重)和 imu0 的 differential yaw 双重计入同一陀螺仪信号,已移除。`base_link → imu_link` 静态 TF 在 `base.launch.py` 里发布(缺失时 robot_localization 会静默丢弃全部 IMU 测量)。

3D 模式下 Gemini 2L 的 IMU 流通过 launch 参数关掉(`enable_accel/gyro: false`),节省 USB 带宽和 CPU。

## Calibration

TF tree calibration plan (wheel odometry → IMU → camera) is documented in `docs/calibration.md`. Wheel effective parameters and gyro scale/bias handling have prior calibration work; IMU mounting refinement remains. The previous camera extrinsic is invalid because its gimbal was removed, so Part 3 must be repeated only after the final camera bracket is fixed.

~~Known issue: missing `base_link → imu_link` static TF~~ — **已修复**,`base.launch.py` 现在发布该 TF(平移是估计值,轴向对齐才关键;若控制板装歪需改 RPY)。Part 2 标定时精化。

## Hardware Serial Protocol

Full protocol documented in `docs/hardware_protocol.md`. Critical details:

- **Device:** `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B21250490-if00` @ 1,000,000 baud (枚举为 `/dev/ttyACMx`,但用 by-id 稳定路径)。Must set `rts=False, dtr=False` before opening.
- **Packet:** `[0xAA] [0x55] [Function] [Length] [Data...] [CRC8]`. CRC8 uses lookup table (not bit-by-bit), computed over Function+Length+Data.
- **Motor IDs are 0-indexed** in the protocol (motor 1 sends as 0). Speed is float32 LE in rps.
- **Motors 1,2 are sign-inverted** in the mecanum kinematics (official SDK convention).
- **Servo IDs are 1-based.** PWM position is uint16 LE (500-2500 μs range).
- **IMU (Function=7):** STM32 auto-reports. 24 bytes = 6×float32 LE (ax,ay,az in g; gx,gy,gz in deg/s).
- **Mecanum parameters:** 物理尺寸 wheelbase=0.1368m, track_width=0.1410m, wheel_diameter=0.065m（标称）。**代码用标定后的有效值**：wheelbase=0.1528/track_width=0.1575（2026-07-16 对墙旋转标定，含原地旋转打滑）、wheel_diameter=0.0636（2026-07-05 卷尺标定）、gyro_scale_z=0.9930——base_node 与 mecanum.xacro 必须同步。

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

**增量建图(多会话)**:slam_3d 对已存在的 db 是**追加式**——同一个 `database_path` 再次进入即"续图",新旧会话靠回环合并。续图建议加 `load_all_nodes:=true`(旧图节点全载入 WM,开头即重定位合并,不用漂移等回环)。**建新图 = 传新文件名**(`database_path:=~/rtabmap_maps/room2.db`),不必手动备份旧库。

**注意:** `addWordRef() Not found word` / `loadWordsQuery ... loaded words (0)` = 词典损坏(典型原因:落库中途被杀,supervisor 已加 90s 宽限防它)。修复:`rtabmap-reprocess corrupted.db repaired.db`(从库存图像重建词典,回环全保留);或备份重扫。

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
- If camera fails to initialize (`uvc_open -6` / `openUsbDevice failed`), `camera_watchdog` now auto-recovers (usbreset + driver restart, ~1min cycle)。手动兜底: `usbreset 2bc5:0670` 后重启 driver。Stale `component_container` processes from a previous launch can also hold the device — check with `ps -ef | grep component_container`.

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

固定 TF 全部来自 `mentorpi_description/urdf/mecanum.xacro`(robot_state_publisher 发布,不再用 static_transform_publisher):

| Joint | Parent → Child | xyz (m) | rpy (rad) | Notes |
|-------|----------------|---------|-----------|-------|
| `camera_joint` | `base_link → camera_link` | 0.1114 0.0305 0.0950 | -0.0164 -0.1302 0.0147 | **失效历史值**：对应已拆除的 2-DOF 云台；最终支架固定后用 `calibrate_camera_extrinsic.py --update-xacro` 重标 |
| `laser_joint` | `base_link → laser_frame` | -0.012242 0 0.092501 | 0 0 0 | 从标定前 xacro + STL 恢复的直装变换；扫描面离地 143.001 mm |
| `imu_joint` | `base_link → imu_link` | 0 0 0.05 | 0 0 0 | 估计值,Part 2 标定精化 |

平移误差 → 点云整体偏移;旋转误差 → 点云累积"分层"伪影。改完 xacro 需 `colcon build --packages-select mentorpi_description` 后重启 base。

## Extending the System

### Adding navigation (Nav2)

The system provides all inputs Nav2 needs:
- `/map` or `/rtabmap/grid_map` — costmap source
- `odom → base_link` TF — robot localization (EKF)
- `/scan` — obstacle detection
- `/cmd_vel` — velocity commands

### Switching between 2D and 3D modes

The two modes share the same EKF-based frontend (base_node + STM32 IMU madgwick + ekf_node) and must not run simultaneously — both publish `map → odom` TF (slam_toolbox vs rtabmap), which would conflict.

**生产用法走 `mentorpi_supervisor`**:`base.launch.py` 常驻不动,supervisor 通过 subprocess 启停模式专用 launch (`slam_2d` / `slam_3d` / `loc_2d`)。切换不重启 base,里程计/TF 连续。

主要差别:
- **2D SLAM**:lidar `/scan` → slam_toolbox 出 `map → odom`
- **3D SLAM**:Gemini 2L RGB-D **+ lidar `/scan`** → rtabmap 出 `map → odom` + 累积彩色点云。scan 用于 `RGBD/NeighborLinkRefining`(相邻节点 ICP 修正轮速打滑)和 `RGBD/ProximityBySpace`(重访区域的激光 proximity link,补视觉 loop closure 在白墙/暗光下的盲区)
- **2D Localization**:slam_toolbox loc 模式 + 已有 posegraph
- **3D Localization (loc_3d)**:rtabmap 只读定位模式 + 已有 .db,重启后恢复 map 系位姿
- **idle**:无 SLAM,纯遥控+预览

## Remote Operation

`ros2 launch mentorpi_supervisor remote.launch.py` 在 base 之上额外起 5 个进程,同时支持桌面客户端和移动客户端:

| 端口 | 服务 | 客户端 |
|------|------|--------|
| 8000 | python `http.server` (静态 SPA) | 手机/平板 浏览器 |
| 8081 | `web_video_server` (MJPEG) | 手机/平板 浏览器 (`<img>`) |
| 9090 | `rosbridge_websocket` | 手机/平板 SPA (roslibjs) |
| 8765 | `foxglove_bridge` | 桌面 Foxglove Studio |

依赖一次性安装:
```bash
sudo apt install -y ros-jazzy-foxglove-bridge ros-jazzy-rosbridge-suite ros-jazzy-web-video-server ros-jazzy-image-transport-plugins
```

`image-transport-plugins` 提供 `compressed_image_transport`,orbbec driver 装上后会自动多发一路 `/camera/color/image_raw/compressed` (JPEG, ~0.5-1 Mbps)。Foxglove Image 面板订这个,WiFi 带宽从 ~5 Mbps 原始 MJPG 降到能容下控制流(否则 WebSocket 拥塞会拖慢 cmd_vel 等所有 topic)。

### 桌面 (Foxglove Studio)

浏览器开 [studio.foxglove.dev](https://studio.foxglove.dev/) 或下载 desktop app,连 `ws://<robot-ip>:8765`,File → Import layout → 选 `src/mentorpi_supervisor/foxglove_layout/mentorpi.json`。

注意:Foxglove web (https) 连 `ws://` 会被浏览器 mixed-content 拒绝。要么用 desktop app,要么 docker 自托管 studio web build (HTTP),要么给 foxglove_bridge 加 TLS。

### 移动 (手机 / 平板)

浏览器直接打开 `http://<robot-ip>:8000/` 即可。SPA 自连 9090 (控制) + 8081 (视频),布局针对竖屏触屏优化:状态条 + MJPEG 视频 + 模式按钮 + 双虚拟摇杆 (左 linear x/y,右 angular z) + 地图选择 sheet。代码在 `src/mentorpi_supervisor/web/`,vendor 自托管 `roslibjs` + `nipplejs`,**手机端无外网即可工作**。

摇杆默认上限 `MAX_LIN=0.4 m/s`、`MAX_ANG=1.5 rad/s` (见 `web/app.js`),发布 20Hz,松手 300ms 自动归零 (deadman)。

### 开机自启 (systemd)

```bash
# 安装 + 立即启动 + 开机自启:
bash scripts/install-systemd.sh

# 查看状态 / 日志:
systemctl status mentorpi-remote
journalctl -u mentorpi-remote -f

# 临时停 / 永久卸载:
sudo systemctl stop mentorpi-remote
bash scripts/install-systemd.sh disable
```

unit 文件在 `scripts/mentorpi-remote.service`,关键点:
- `User=pi`, `SupplementaryGroups=dialout video plugdev input` (访问 ttyACM/ttyUSB/Gemini)
- `PATH=/usr/bin:/usr/local/bin:/opt/ros/jazzy/bin` 显式排除 conda (否则 rclpy ABI 撞)
- `bash -c` 非交互非登录,不读 `~/.bashrc`,绕开 conda auto-activate
- `Restart=on-failure RestartSec=5` + 5次/60秒限流——开机时 ttyACM0 还没枚举会自动重试
- `KillSignal=SIGINT` 让 ros2 launch 优雅传播给所有节点

成功启动后底盘蜂鸣器会响 3 短声 (~0.5 秒)——听到就说明 base + supervisor 都起来了,可以连客户端。

**Supervisor 接口**:

| Endpoint | Type | Purpose |
|----------|------|---------|
| `/mode/set` | `mentorpi_msgs/srv/SetMode` | 切换模式 (`idle`/`slam_2d`/`slam_3d`/`loc_2d`/`loc_3d`) |
| `/mode/list_maps` | `mentorpi_msgs/srv/ListMaps` | 列出 `~/maps/*.posegraph` 或 `~/rtabmap_maps/*.db` |
| `/mode/status` | `std_msgs/String` (transient_local) | 当前模式名,latched |
| `/buzzer` | `mentorpi_msgs/Buzzer` | supervisor 在 base_node 上线后发一声 "ready" 哔哔 (3×80ms@1900Hz);用 launch arg `enable_startup_chime:=false` 关掉 |

**SetMode 字段**:
- `mode`:必填
- `map_file`:`loc_2d` 必填(slam_toolbox 路径不带扩展名,如 `/home/pi/maps/my_room`)
- `database_path`:`slam_3d`/`loc_3d` 可选,默认 `~/rtabmap_maps/rtabmap.db`
- `load_all_nodes`:`slam_3d` 增量续图用——旧图节点全载入 WM,开局即重定位合并(其余模式忽略)

**安全模型**:第一版裸跑,假设局域网可信(无认证)。生产部署需要在 foxglove_bridge 前加 nginx + TLS + basic auth,或开 `ros-jazzy-foxglove-bridge` 的 TLS。

**视频源**:Gemini 2L 在 base 里常驻,`/camera/color/image_raw` 跨所有模式始终可订。**Foxglove Image 面板必须订 `/camera/color/image_raw/compressed`**(layout 默认配置已切换);订原始 raw 流会让 WebSocket 在 WiFi 上严重拥塞。

**地图保存**(v1 暂不在 supervisor 内):
- 2D: `ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/pi/maps/my_room'}"`
- 3D: rtabmap 边走边自动写 `~/rtabmap_maps/rtabmap.db`

## Motion Primitives（语音 / VLA / agent 的运动执行底座）

`mentorpi_motion/motion_node`（`base.launch.py` 常驻）提供**有界**运动原语，远端智能（语音 agent、VLA）永远发"意图"而不是裸 cmd_vel 流：

- Action `motion/primitive`（`mentorpi_msgs/action/MotionPrimitive`）：`type` = `forward`/`strafe`/`rotate`，`distance` 米或弧度（带符号），`max_speed`、`timeout` 可选。节点自己以 20Hz 发 `/cmd_vel`（喂 base_node watchdog），用 `/odometry/filtered` 闭环测位移，梯形速度曲线，完成/取消/超时/odom 停更 (>0.5s) 时**一律停车**。单 goal 互斥（忙时拒绝新 goal）。
- Service `motion/stop`（`std_srvs/Trigger`）：取消当前 goal + 清零速度，接"停"类指令。
- 每 goal 上限：平移 3m、旋转 ~2 圈；速度上限 0.4 m/s / 1.5 rad/s（参数可调）。
- 测试：`src/mentorpi_motion/test/test_motion_node.py` 用假底盘闭环仿真，dev 机可跑（无需硬件）。
- **rclpy 坑**（已修，勿回退）：Jazzy 的 `goal_handle.succeed()/abort()/canceled()` 会立即用**空 Result** 填 result future，靠 execute 回调返回值回填会跟客户端 result 请求竞态（高负载下客户端拿到空结果）。必须把 result 作为参数传给终态调用（见 `MotionNode._finalize`）。

**语音接入**：AsynchronousIntentRoutingEngine（独立仓库）的 `robot` skill 通过 rosbridge (:9090) 调上述接口；服务器端设 `AIR_ROBOT_ROSBRIDGE_URL=ws://<robot-ip>:9090` 即启用。手动验证：AIRE 仓库 `python robot_cli.py ws://<robot-ip>:9090 move forward 0.5`。整体规划见 `docs/roadmap.md`。

## SO-101 机械臂集成（layout v3，甲板 CAD 已定型、待实物装配）

当前执行入口是 `docs/so101_handoff.md`；设计摘要和几何账本分别见
`mechanical/README.md`、`mechanical/urdf/design-ledger.md`。旧 integration
report 和 layout v2 提交只用于追溯，不再作为待办清单。

- **臂朝前**，`base_link -> so101_base_link xyz=-0.155 0 0.0655`。恢复雷达
  直装 TF 后，真实网格 pan ±110° 全扫转最差雷达头间隙 36.2 mm。
- **雷达直接贴装**，`laser_joint xyz=-0.012242 0 0.092501`，扫描面离地
  143.001 mm；没有塔架或增高座。甲板止于 x=-0.050，与雷达保留
  10.95 mm 平面间隙。
- **甲板四孔**：底盘 x=-61 mm，y=-24/-8/+8/+24 mm，Ø4.5 打印通孔，
  M4 螺丝 + 垫圈 + 螺母。用户已确认孔无螺纹、贯穿且不固定雷达。
- **自体掩膜**：装臂后 oradar 发 `/scan_raw`，`laser_filters` 按真实几何
  掩膜后向 ±24° 后发 `/scan`。部署依赖 `ros-jazzy-laser-filters`。
- **启用开关**：`with_so101:=true` 从 remote.launch.py 透传到 base.launch.py
  和 xacro；默认 false 不加入臂、甲板和扫描滤波器。
- **相机外参待重标**：旧 2-DOF 云台已拆除，当前 `camera_joint` 只是历史值；
  最终支架固定后才运行 AprilTag 标定。
- **供电**：独立 2S 锂电 + 保险丝 + 带锁开关到 Feetech 板。前托架的
  `HOOK_THROAT` 和 `PACK_*` 仍需用实物尺寸更新。
- 参数化源、派生物、验证命令和下一步顺序全部维护在当前交接文档中。

## 3D 可视化 & 高斯泼溅 (Gaussian Splatting)

完整管线见 `docs/gaussian_splatting.md`。要点:

- `scripts/export_gs_dataset.py`:rtabmap.db → nerfstudio 数据集(回环优化后相机位姿 + RGB/depth + 种子点云),**保持 ROS map 坐标系**
- 服务器训练 splatfacto 必须加 `--orientation-method none --center-method none --auto-scale-poses False`,导出的 splat.ply 才和 SLAM 地图天然对齐
- `scripts/live_rerun.py`(跑在查看端 PC,Pi 零负担):Rerun 里显示 splat/SLAM 点云 + 实时机器人位姿(TF via rosbridge :9090)+ 相机 FOV 视锥 + 实时视频;`--serve` 供手机浏览器
- 重启后流程:切 `loc_3d` 模式 → rtabmap 重定位发布 map→odom → live_rerun 里机器人出现在 splat 场景中的真实位置
- `scripts/bag_to_rerun.py`:离线 bag 回放调试(不变)

## Known Issues

**USB 过流导致外设集体掉线(遥控一段时间后崩溃)** — 根因、供电拓扑、台架持续调试配置见 `docs/power_troubleshooting.md`。

### STM32 蜂鸣器在 Pi 接官方 PSU 时持续 5 声循环报警

**症状**:Pi 由 STM32 共享的 Type-C 供电时一切正常;改用官方原装 PSU 单独给 Pi 供电后,STM32 蜂鸣器开始 5 声 × 1 秒循环报警。SDK 主机侧不发任何 5 声序列(已 grep 确认),所以这是 STM32 固件本身的报警。

**最可能根因**(尚未实测确认):
1. **反灌**:STM32 → Pi 的 Type-C 共享线没拔,Pi 被官方 PSU 推到 5.1V,反向倒灌进 STM32 输出端,固件触发保护。
2. **接地环路**:官方 PSU 接市电地,STM32 在电池上(浮地),USB 串口线的 GND 把两个参考地连起来,几百 mV 的电位差让 STM32 ADC 误读自己的电池电压,触发欠压报警(5 声 1Hz 是常见 lipo 欠压模式)。

**判别方法**:用官方 PSU 时,把 STM32 → Pi 的 Type-C 共享线拔掉,只保留 USB 串口线。
- 5 声没了 → 反灌
- 还在响 → 接地环路(需要 USB 隔离器,或使用接地良好的 PSU)

短期解决:就用 STM32 共享供电(电池模式),或拔掉 Type-C 共享线只走官方 PSU + 串口线。

## Reference Code

Official vendor SDK is at `/home/pi/workdir/mentorpi/src/` — use it as reference for protocol details but don't modify it. Key files:
- `ros_robot_controller_sdk.py` — authoritative serial protocol (including IMU: `get_imu()`, `packet_report_imu()`)
- `ros_robot_controller_node.py` — official IMU publishing (`pub_imu_data()`)
- `controller/mecanum.py` — official mecanum kinematics
- `peripherals/joystick_control.py` — official joystick mapping
- `p2117_ros/oradar_ros/` — lidar driver source (copied to our `src/oradar_lidar/`)