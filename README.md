# MentorPi ROS 2 Workspace

基于 **ROS 2 Jazzy** 的机器人控制系统，适配 MentorPi 硬件（树莓派 5 + RRCLite STM32 麦克纳姆轮底盘 + 2-DOF 云台 + MS200 激光雷达 + Orbbec Gemini 2L 深度相机）。

---

## 快速启动

### 编译

```bash
cd ~/workdir/mentorpi/mentorpi_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### 启动模式

| 模式 | 命令 | 说明 |
|------|------|------|
| **远程操作（生产入口）** | `ros2 launch mentorpi_supervisor remote.launch.py` | base 常驻 + foxglove_bridge + rosbridge + 手机 SPA + supervisor；之后用 `/mode/set` 服务在 `idle`/`slam_2d`/`slam_3d`/`loc_2d`/`loc_3d` 间热切换，不重启 base（开机自启见 `scripts/install-systemd.sh`） |
| **基础遥控** | `ros2 launch mentorpi_bringup mentorpi.launch.py` | 底盘 + 手柄 + 2D 激光雷达 + IMU/EKF 融合 |
| **2D 建图** | `ros2 launch mentorpi_bringup mapping.launch.py` | 基础遥控 + slam_toolbox 2D SLAM |
| **2D 定位** | `ros2 launch mentorpi_bringup localization.launch.py` | 基础遥控 + 加载已有 2D 地图定位 |
| **3D 建图** | `ros2 launch mentorpi_bringup rtabmap_mapping.launch.py` | EKF (轮速+IMU) 前端 + RTAB-Map (Gemini 2L RGB-D) 后端，异构架构 |

supervisor 模式切换示例（生产用法，地图文件按需替换）：

```bash
# 3D 建图到指定库（新文件名 = 新图；同名 = 增量续图，可加 load_all_nodes: true）
ros2 service call /mode/set mentorpi_msgs/srv/SetMode \
  "{mode: slam_3d, database_path: /home/pi/rtabmap_maps/room.db}"
# 3D 定位（重启后恢复 map 系位姿）
ros2 service call /mode/set mentorpi_msgs/srv/SetMode \
  "{mode: loc_3d, database_path: /home/pi/rtabmap_maps/room.db}"
# 停止 SLAM 回到纯遥控
ros2 service call /mode/set mentorpi_msgs/srv/SetMode "{mode: idle}"
```

### 依赖安装

```bash
# 2D SLAM
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-imu-filter-madgwick ros-jazzy-robot-localization

# 3D SLAM (额外)
sudo apt install ros-jazzy-rtabmap-ros

# RViz2 IMU 可视化插件 (远程 PC)
sudo apt install ros-jazzy-rviz-imu-plugin
```

---

## 系统架构

### 分层设计

```
┌────────────────────────────────────────────────────────────────┐
│ 输入层        joy_node (手柄)  /  App  /  Nav2                  │
├────────────────────────────────────────────────────────────────┤
│ 控制层        mentorpi_teleop → /cmd_vel, /gimbal/cmd          │
├────────────────────────────────────────────────────────────────┤
│ 感知融合层    IMU Filter (Madgwick) → EKF (前端，所有模式共用)  │
│               slam_toolbox (2D) / RTAB-Map (3D 后端)            │
├────────────────────────────────────────────────────────────────┤
│ 驱动层        mentorpi_base (串口)  oradar_lidar  orbbec_camera│
├────────────────────────────────────────────────────────────────┤
│ 硬件层        RRCLite STM32 (电机+舵机+IMU)                    │
│               MS200 激光雷达  Gemini 2L 深度相机                │
└────────────────────────────────────────────────────────────────┘
```

### 2D SLAM 数据流

```
                    mentorpi.launch.py
                    ┌──────────────────────────────────────────────┐
joy_node → teleop → /cmd_vel → base_node ──→ Serial → STM32      │
                                   │                               │
                                   ├→ /odom (dead-reckoning)       │
                                   └→ /imu/data_raw (STM32 IMU)   │
                                          │                        │
                                     madgwick                      │
                                          │                        │
                                     /imu/data                     │
                                          │                        │
                              /odom ──→ EKF ──→ TF: odom→base_link│
                                               → /odometry/filtered│
                    └──────────────────────────────────────────────┘

oradar_scan → /scan ──→ slam_toolbox ──→ TF: map→odom
                                       → /map (OccupancyGrid)
```

### 3D SLAM 数据流（异构架构）

**前端**（高频 dead-reckoning，与 2D 模式共用）：

```
base_node ─→ /odom (50Hz, cmd_vel 积分) ─────┐
          ─→ /imu/data_raw (STM32 IMU)       │
                       │                      │
                  madgwick ↓                  │
                  /imu/data                   │
                       │                      ↓
                       └────────────→ EKF ←──/odom
                                       │
                                       ├─→ TF: odom → base_link (50Hz)
                                       └─→ /odometry/filtered
```

**后端**（低频建图 + 周期性 loop closure）：

```
Gemini 2L ─→ /camera/color/image_raw ────┐
           → /camera/depth/image_raw ─────┤→ rtabmap (2Hz) ──→ TF: map → odom
           → /camera/color/camera_info ───┤    ↑               → /cloud_map (累积彩色点云)
                                          │    │               → /rtabmap/cloud (当前帧点云)
                                          │    │               → /map (2D 栅格)
                                          │    └── TF lookup (odom→base_link by EKF)
                                          │
oradar_scan ─→ /scan (避障用，不参与 3D 建图)
```

> **关键差别**（对比旧的纯视觉里程计设计）：
> - **没有 `rgbd_odometry`**：机器人位姿来自 EKF 融合（轮速+STM32 IMU），~20ms 延迟，45Hz
> - **没有相机 IMU**：Gemini 2L 内置 IMU 不参与（`enable_accel/gyro: false`）
> - **rtabmap 通过 TF 拿位姿**：`subscribe_odom_info: false`，从 TF 链 `odom → base_link` 读
> - **轮速里程计漂移由 loop closure 校正**：rtabmap 在 `map → odom` 上发出修正

### TF 树

```
2D SLAM:   map → odom → base_link → laser_frame
        (slam_toolbox) (EKF)        (static z=0.18m)

3D SLAM:   map → odom → base_link → camera_link → camera_*_optical_frame
        (rtabmap)   (EKF)         (static)        (orbbec driver)
                                   → laser_frame (static z=0.18m)
```

`base_link → camera_link` 静态 TF 在 `rtabmap_mapping.launch.py` 中手填（默认 x=0.05, z=0.15，**只是估计值**），实际需用尺子量后校准；如有俯仰角填 RPY 的 pitch（弧度制，10° ≈ 0.1745）。

---

## 软件包

### 自研包

| 包名 | 类型 | 节点 | 职责 |
|------|------|------|------|
| `mentorpi_msgs` | ament_cmake | — | 自定义消息：`Gimbal.msg`, `MotorStatus.msg` |
| `mentorpi_base` | ament_python | `base_node` | 串口通信、麦轮运动学、里程计、IMU 读取 |
| `mentorpi_teleop` | ament_python | `teleop_node` | 手柄映射（北通 BTP-KP20D） |
| `mentorpi_vision` | ament_python | `vision_node` | OpenCV 相机图像发布 |
| `mentorpi_bringup` | ament_python | — | Launch 文件 + 配置文件 |
| `oradar_lidar` | ament_cmake | `oradar_scan` | MS200 激光雷达驱动 |

### 外部依赖包

| 包名 | 节点 | 用于 | 安装 |
|------|------|------|------|
| `slam_toolbox` | `async_slam_toolbox_node` | 2D SLAM | `apt: ros-jazzy-slam-toolbox` |
| `imu_filter_madgwick` | `imu_filter_madgwick_node` | IMU 姿态估计 | `apt: ros-jazzy-imu-filter-madgwick` |
| `robot_localization` | `ekf_node` | 多源里程计融合 | `apt: ros-jazzy-robot-localization` |
| `rtabmap_ros` | `rtabmap`, `point_cloud_xyzrgb` | 3D SLAM 后端 + 彩色点云 | `apt: ros-jazzy-rtabmap-ros` |
| `orbbec_camera` | Gemini 2L 驱动 | 深度相机 | 已预装 |

### Launch 文件

| 文件 | 包含内容 |
|------|---------|
| `mentorpi.launch.py` | base_node + STM32 IMU Madgwick + EKF + 相机 + 手柄 + 遥控 + 激光雷达 |
| `mapping.launch.py` | mentorpi.launch.py + slam_toolbox 2D 建图 |
| `localization.launch.py` | mentorpi.launch.py + slam_toolbox 定位模式 |
| `rtabmap_mapping.launch.py` | base_node + STM32 IMU Madgwick + EKF + Gemini 2L (RGB-D only) + RTAB-Map（异构架构） |

### 配置文件 (`src/mentorpi_bringup/config/`)

| 文件 | 说明 |
|------|------|
| `imu_filter.yaml` | STM32 IMU Madgwick 滤波参数 |
| `ekf.yaml` | EKF 融合参数（/odom + /imu/data） |
| `slam_toolbox_params.yaml` | 2D SLAM 建图参数 |
| `slam_toolbox_localization_params.yaml` | 2D SLAM 定位参数 |

---

## 各模式详细说明

### 基础遥控

```bash
ros2 launch mentorpi_bringup mentorpi.launch.py
# 使用深度相机替代 OpenCV 相机：
ros2 launch mentorpi_bringup mentorpi.launch.py camera_type:=gemini2l
```

手柄操作：
- **左摇杆**：前后 (ly→linear.x) / 横移 (lx→linear.y)
- **右摇杆 X**：旋转 (angular.z)
- **按住 RB + 右摇杆**：云台控制（松开 RB 自动回中）
- 死区：0.1

### 2D 建图 (slam_toolbox)

```bash
# 启动建图
ros2 launch mentorpi_bringup mapping.launch.py

# 遥控小车走一圈后保存地图
mkdir -p ~/maps
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/pi/maps/my_room'}"
```

生成文件：`my_room.posegraph` + `my_room.data`

### 2D 定位 (加载已有地图)

```bash
# 默认地图
ros2 launch mentorpi_bringup localization.launch.py

# 指定地图（不含扩展名）
ros2 launch mentorpi_bringup localization.launch.py map_file:=/home/pi/maps/kitchen
```

### 3D 建图 (RTAB-Map + Gemini 2L)

```bash
# 创建地图存储目录
mkdir -p ~/rtabmap_maps

# 启动 3D 建图
ros2 launch mentorpi_bringup rtabmap_mapping.launch.py

# 地图自动保存到 ~/rtabmap_maps/rtabmap.db
# 指定其他路径：
ros2 launch mentorpi_bringup rtabmap_mapping.launch.py \
  database_path:=~/rtabmap_maps/kitchen.db
```

### RViz2 远程可视化

在远程 PC 上：

```bash
# 建议两端统一使用 Cyclone DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rviz2
```

**RViz2 配置：**

| 显示项 | 类型 | Topic | 适用模式 |
|--------|------|-------|----------|
| 2D 激光 | LaserScan | `/scan` (QoS: Best Effort) | 所有 |
| 2D 地图 | Map | `/map` | 所有 SLAM |
| 当前帧 3D 点云 | PointCloud2 | `/rtabmap/cloud` | 3D SLAM |
| **累积 3D 彩色地图** | PointCloud2 | `/cloud_map` | 3D SLAM |
| 3D 累积地图（插件） | MapCloud (rtabmap plugin) | `/mapData` | 3D SLAM |
| 位姿图 | MapGraph (rtabmap plugin) | `/mapGraph` | 3D SLAM |
| 里程计轨迹 | Odometry | `/odometry/filtered` (EKF 输出) | 所有 |
| IMU 姿态 | Imu (需 rviz-imu-plugin) | `/imu/data` (Madgwick 滤波后) | 所有 |
| 坐标系 | TF | — | 所有 |

Fixed Frame 设为 `map`（建图/定位时）或 `odom`（无 SLAM 时）。

### 3D 查看方式一览

| 方式 | 客户端 | 连接 | 特点 |
|------|--------|------|------|
| **Foxglove Studio** | 桌面 app | `ws://<robot-ip>:8765` | 日常主力；导入 `src/mentorpi_supervisor/foxglove_layout/mentorpi.json` 布局；图像面板务必订 `/camera/color/image_raw/compressed` |
| **手机 SPA** | 手机/平板浏览器 | `http://<robot-ip>:8000/` | 虚拟摇杆遥控 + MJPEG 视频 + 模式切换，无外网可用 |
| **RViz2** | 远程 PC | DDS 直连 | 见上节配置表 |
| **Rerun (live_rerun.py)** | 查看端 PC / 手机 | foxglove :8765（默认）或 rosbridge :9090 | 3DGS splat / SLAM 点云 + 实时机器人位姿 + 相机视锥 + 视频，Pi 零负担 |

```bash
# dev 机（venv 需 rerun + rosbags + websockets + numpy）：
python scripts/live_rerun.py --robot <robot-ip> \
    --cloud gs_work/gs_dataset_xxx/sparse_pc.ply      # 点云版
python scripts/live_rerun.py --robot <robot-ip> \
    --splat gs_work/exports/splat/splat.ply           # 3DGS 版
# 加 --serve 可用手机浏览器打开（URL 里的 + 要编码为 %2B）
```

机器人切 `loc_3d` 模式后，rtabmap 重定位发布 `map→odom`，Rerun 里的机器人会出现在 splat/点云场景中的真实位置。

### 3D 重构：高斯泼溅 (3DGS) 管线

完整文档见 **`docs/gaussian_splatting.md`**。四步走，坐标系全程锁定 ROS `map` 系（训练出的 splat 与 SLAM 地图天然对齐，重定位即可把机器人"放进"场景）：

```bash
# 1. 建图（机器人上，slam_3d 模式慢速多角度扫，db 自动落盘）

# 2. 导出 nerfstudio 数据集（dev 机，需 ros-jazzy-rtabmap 的 rtabmap-export）
python3 scripts/export_gs_dataset.py <path>/room.db --output-dir gs_work/gs_dataset_room
#    产物：transforms.json（回环优化后位姿）+ images/ + depth/ + sparse_pc.ply（种子点云）

# 3. 训练 splatfacto（nerfstudio 环境；三个 flag 缺一不可——保持 map 坐标系）
ns-train splatfacto --data gs_work/gs_dataset_room \
    --viewer.quit-on-train-completion True \
    nerfstudio-data --orientation-method none --center-method none --auto-scale-poses False

# 4. 导出 splat.ply 并查看
TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1 ns-export gaussian-splat \
    --load-config outputs/gs_dataset_room/splatfacto/<timestamp>/config.yml \
    --output-dir exports/splat/
python scripts/live_rerun.py --robot <robot-ip> --splat exports/splat/splat.ply
```

注意：相机外参重标后旧 db 的几何随之失效，重训 splat 应使用**新外参重扫的库**（换新 db 文件名，别在旧库上增量）。离线调试可用 `scripts/bag_to_rerun.py` 回放 bag。

---

## 硬件说明

### 串口通信协议

帧格式：`[0xAA] [0x55] [Function] [Length] [Data...] [CRC8]`

| 功能码 | 名称 | 方向 | 说明 |
|--------|------|------|------|
| 3 | MOTOR | 发送 | 电机速度控制（float32 rps） |
| 4 | PWM_SERVO | 发送 | 云台舵机（uint16 脉宽 500-2500μs） |
| 7 | IMU | 接收 | 6×float32：ax,ay,az (g) + gx,gy,gz (deg/s) |

- **设备**：`/dev/ttyACM0` @ 1,000,000 baud
- **初始化**：`rts=False, dtr=False`，打开后等待 0.5s
- **电机 ID**：协议 0-indexed（代码中 1-based，发送时减 1）
- **电机 1,2 取反**（官方 SDK 约定）
- **IMU 自动上报**，无需使能命令

完整协议见 `docs/hardware_protocol.md`。

### 底盘参数

| 参数 | 物理尺寸 | 代码用值（标定后） |
|------|---------|------------------|
| 前后轴距 (wheelbase) | 0.1368 m | **0.1528 m**（2026-07-16 对墙旋转标定，有效几何含麦轮原地旋转打滑） |
| 左右轴距 (track_width) | 0.1410 m | **0.1575 m**（同上） |
| 轮径 (wheel_diameter) | 0.065 m（标称） | **0.0636 m**（2026-07-05 卷尺标定） |
| 陀螺 z 刻度 (gyro_scale_z) | — | **0.9930**（2026-07-16） |

标定值在 `mentorpi_base/base_node.py` 与 `mentorpi_description/urdf/mecanum.xacro` 中**必须保持同步**；标定方法见 `docs/calibration.md` 与 `docs/calibration_handoff.md`。

### 设备列表

| 设备 | 端口 | 波特率 | 说明 |
|------|------|--------|------|
| RRCLite STM32 | `/dev/ttyACM0` | 1,000,000 | 底盘+云台+IMU |
| MS200 激光雷达 | `/dev/ttyUSB0` | 230,400 | CH340 转换器 |
| Gemini 2L 深度相机 | USB | — | RGB-D + 内置 IMU |
| 北通手柄 (BTP-KP20D) | USB 无线 | — | 2.4G dongle |

### 已知问题

- `/dev/ttyACM0` 偶尔重启后消失，重启 Pi 可解决
- `oradar_scan` 节点 Ctrl+C 可能无法干净退出，用 `pkill -9 -f oradar_scan`
- 跨网络 RViz2 看不到数据时，两端都设置 `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- RTAB-Map 在低纹理环境（白墙、纯色地面）loop closure 检测会失败，机器人位姿改靠纯 dead-reckoning（轮速+IMU），漂移会累积直到回到有特征区域才被校正
- Gemini 2L **必须插 USB 3.0 蓝口** + USB-C 3.0 线，USB 2.0 会导致 `color frame is not decoded` 和频繁断连
- 重复启动 launch 后，老进程可能残留持有 USB 设备导致相机初始化失败；用 `usbreset 2bc5:0670` 重置相机

### Gemini 2L 稳定性配置

Pi 5 高负载时 USB 控制器对电压敏感，Gemini 2L 的 IR 投射器上电瞬态容易导致掉线。建议：

1. **EEPROM 允许 5A 输出**
   ```bash
   sudo rpi-eeprom-config --edit
   # 加一行：
   PSU_MAX_CURRENT=5000
   ```

2. **解除 USB 限流**：`/boot/firmware/config.txt` 加
   ```
   usb_max_current_enable=1
   ```

3. **必须用官方 27W 电源 + 原装 USB-C 3.0 线**。

4. 重启后验证：
   ```bash
   lsusb -t | grep uvcvideo   # 应看到 5000M，不是 480M
   vcgencmd get_config usb_max_current_enable   # 应返回 1
   ```

---

## 扩展指南

### 性能调优（Pi 5 实测）

3D 模式下经过下面这些优化，新架构在 Pi 5 上可达：**机器人 pose 延迟 ~20ms / 45Hz**，rtabmap 建图 2Hz，核心 CPU 占用约 75%（4 核共 400%）。

Gemini 2L launch 参数：

| 参数 | 值 | 说明 |
|------|-----|------|
| `color_width/height/fps` | `640/480/15` | 30→15 fps 大幅降低 USB+driver CPU |
| `depth_width/height/fps` | `640/400/15` | 同上 |
| `color_format` | `MJPG` | YUYV 在 Pi 5 上无收益 |
| `enable_accel/gyro` | `false` | 异构架构不用相机 IMU |
| `enable_colored_point_cloud` | `false` | driver 端的彩色点云生成关闭，改用独立 `point_cloud_xyzrgb` |
| `depth_registration` | `true` | HW D2C 对齐（ASIC 内做，host 不参与） |

`point_cloud_xyzrgb` 节点：`decimation: 8`，`voxel_size: 0.10`。

rtabmap 节点：`subscribe_odom_info: false`，`Rtabmap/DetectionRate: 2.0`，`Kp/MaxFeatures: 300`，`topic_queue_size: 20`。

### 接入 Nav2 导航

系统已提供 Nav2 所需的全部输入：
- **地图**：`/map`（2D）或 `/rtabmap/grid_map`（3D）
- **定位 TF**：`map → odom → base_link`（两种模式都由 EKF 提供 `odom → base_link`）
- **避障**：`/scan`（激光雷达）
- **控制**：Nav2 输出 `/cmd_vel` 直接驱动底盘

### AI 视觉

1. 订阅 `/camera/color/image_raw`（Gemini 2L）或 `/camera/image_raw`（OpenCV）
2. 处理后发布 `/gimbal/cmd`（`mentorpi_msgs/Gimbal`）实现自动跟随
3. 或发布 `/cmd_vel` 实现视觉导航

---

## 参考

- **官方 SDK**：`/home/pi/workdir/mentorpi/src/`（协议参考，勿修改）
- **硬件协议详情**：`docs/hardware_protocol.md`
- **高斯泼溅 3D 重构管线**：`docs/gaussian_splatting.md`
- **标定总计划 / 实车标定手册**：`docs/calibration.md` / `docs/calibration_handoff.md`
- **供电排障**：`docs/power_troubleshooting.md`
- **Claude Code 开发指引**：`CLAUDE.md`
