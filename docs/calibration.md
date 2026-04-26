# MentorPi TF 树标定指南

本文档描述 MentorPi 三部分标定任务，覆盖整个 TF 树中所有需要人工校准的边。

```
map → odom → base_link → camera_link → camera_*_optical_frame
                       → imu_link
                       → laser_frame
```

| TF 边 | 来源 | 是否需要标定 |
|-------|------|-------------|
| `map → odom` | rtabmap / slam_toolbox | 否（算法输出） |
| `odom → base_link` | EKF（融合 `/odom` + `/imu/data`） | **是** — 需要标定轮速里程计 + IMU |
| `base_link → imu_link` | （当前缺失） | **是** — 需要补静态 TF |
| `base_link → camera_link` | static_transform_publisher | **是** — 当前是占位估计值 |
| `base_link → laser_frame` | static_transform_publisher | 否（z=0.18 已实测） |
| `camera_link → camera_*_optical_frame` | Orbbec 驱动 | 否（出厂标定） |

## 标定顺序

**必须按顺序进行**，前两步的结果会影响第三步精度：

```
Part 1 (轮速)  ─┐
                ├─→ Part 3 (相机)  ← 依赖前两步可信的 base_link 运动
Part 2 (IMU)   ─┘
```

如果先标相机后标轮速，相机外参会吸收掉里程计的误差，等里程计修好后相机又错了。

---

## Part 1 — 轮速里程计标定

校准 `src/mentorpi_base/mentorpi_base/base_node.py:281-283` 中硬编码的三个参数：

```python
wheelbase    = 0.1368   # 前后轴距 (m)
track_width  = 0.1410   # 左右轴距 (m)
wheel_diameter = 0.065  # 轮径 (m)
```

### Step 1.1 — 直行标定 `wheel_diameter`

地上贴 5 米卷尺，机器人正向匀速跑 3 米。

```bash
ros2 topic echo /odom --field pose.pose.position.x
```

- 实际走 D 米，odom 报 d 米
- `wheel_diameter_new = wheel_diameter_old × D / d`
- 重复 3 次取平均，反向再跑 3 次确认对称性

### Step 1.2 — 原地旋转标定 `wheelbase + track_width`

mecanum 车原地转向时，`wheelbase + track_width` 的和决定角速度比例（见 base_node.py:285 的 `vp = wz * (wheelbase + track_width) / 2.0`）。

操作：
1. 地上贴参考线，机器人对齐
2. 发指令 `wz = 0.5 rad/s` 持续 25.13 秒（理论转 720°）
3. 量实际转角 θ（激光在墙上投点 / 手机量角器 app）
4. `(wheelbase + track_width)_new = (wheelbase + track_width)_old × 720° / θ`

### Step 1.3 — UMBmark（区分左右轮直径不对称）

跑 4×4m 正方形，**顺时针**和**逆时针**各 5 圈。

| 现象 | 诊断 |
|------|------|
| 两个方向都向内偏 | `track_width` 偏小 |
| 两个方向都向外偏 | `track_width` 偏大 |
| 顺逆方向偏的方向不同 | 左右轮直径不对称（mecanum 车较少见） |

### Step 1.4 — 应用

修改 `base_node.py:281-283` 后：

```bash
colcon build --packages-select mentorpi_base
source install/setup.bash
```

---

## Part 2 — STM32 IMU 标定

> ⚠️ **隐藏 bug**：`base_node.py:163` 把 IMU msg 标 `frame_id='imu_link'`，但 `mentorpi.launch.py` **没有发布 `base_link → imu_link` 静态 TF**。EKF 默认把 IMU 当作在 base_link 帧上，如果 IMU 没和车体完全同向，融合出来的 yaw 会有偏差。**标定时必须补上这条 TF。**

### Step 2.1 — 判断 IMU 安装姿态

让车**静止水平放置**，看 `/imu/data_raw`：

```bash
ros2 topic echo /imu/data_raw --field linear_acceleration
```

期望：`az ≈ +9.81`，`ax, ay ≈ 0`（REP-103：X 前 / Y 左 / Z 上）

| 现象 | 诊断 | 修正 |
|------|------|------|
| `az ≈ -9.81` | IMU Z 轴朝下 | `roll = π` |
| 重力出现在 `ax` 或 `ay` | IMU 装反 90° | 加 yaw 旋转 |

把车**慢慢推一下向前**，看 `linear_acceleration.x` 是不是正的。如果是负的，IMU X 轴朝后，加 `yaw = π`。

### Step 2.2 — 补 `base_link → imu_link` 静态 TF

在 `src/mentorpi_bringup/launch/mentorpi.launch.py` 的 `LaunchDescription` 列表中加：

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_to_imu',
    arguments=['0.0', '0.0', '0.05',   # x y z (实测)
               '0', '0', '0',           # yaw pitch roll (按 Step 2.1 判断)
               'base_link', 'imu_link'],
),
```

### Step 2.3 — Gyro bias

让车**绝对静止 60 秒**，记录 gyro 输出：

```bash
ros2 topic echo /imu/data_raw --field angular_velocity > /tmp/gyro.log
# 60 秒后 ctrl+c
```

平均值就是 bias。理想为 0，实测一般 0.001~0.01 rad/s。

**两种处理方式：**
- 在 base_node 收 IMU 后减掉常数 bias 再发布
- 让 Madgwick 自己估计 bias（开机后让车静止 5-10 秒收敛）

### Step 2.4 — Gyro_z 比例标定

让车原地匀速转 360°（用 1.2 步骤的方法量真实角度 θ），同时积分 `gyro.z`。

- 理想：积分 = 2π
- 不等：在 base_node 里给 gyro.z 乘比例 `2π / 实测积分`

> 注意 base_node 中 IMU 单位转换：accel `g→m/s²`、gyro `deg/s→rad/s`。如果 STM32 固件改了量程（`±2g/±4g/±8g`），转换系数也要改。

---

## Part 3 — Gemini 2L 相机外参标定

校准 `src/mentorpi_bringup/launch/rtabmap_mapping.launch.py` 中 `base_to_camera` 静态 TF（当前是占位估计值 x=0.05, z=0.15, 无旋转）。

**方法选择：AprilTag（精标方法 B）+ 后续自研标定工具**

### 原理

1. 把 AprilTag 贴在地上某个固定位置
2. apriltag_ros 输出 tag 在 `camera_*_optical_frame` 中的 pose（记为 `T_camOptical_tag`）
3. 人工量 tag 在 `base_link` 中的真实 pose（记为 `T_base_tag`）
4. 反推：`T_base_camOptical = T_base_tag × T_camOptical_tag⁻¹`
5. 再经过驱动发布的 `camera_link → camera_*_optical_frame` 反推到 `T_base_cameraLink`

### 依赖

```bash
sudo apt install ros-jazzy-apriltag ros-jazzy-apriltag-ros
```

### 手动验证流程（开发自研工具前先跑通）

1. 打印 AprilTag（推荐 36h11 family，边长 ≥ 10cm），平整贴在地面
2. 启动相机和 apriltag_ros，让相机能稳定看到 tag
3. 量 tag 在 `base_link` 中的位置（用直尺量 tag 中心到 base_link 投影点的 x/y，z=0；姿态按 tag 朝向定）
4. 读 `/tf` 中 `camera_*_optical_frame → tag_<id>` 的变换
5. 用 Python 脚本（tf2 / numpy）做矩阵运算反推 `base_link → camera_link`

### 自研标定工具

后续用户会开发标定工具（暂定包名 `mentorpi_calibration` 或类似），把以上流程自动化：

- 收集多帧 tag 观测取平均（降低单帧噪声）
- 输出 `static_transform_publisher` 的 6 个 launch 参数
- 可选：直接修改 `rtabmap_mapping.launch.py`

> 由于第 3 步依赖 base_link 的运动可信度，**必须先完成 Part 1 + Part 2**，否则相机外参会吸收里程计误差。

---

## 验证整体 TF 树

标完之后：

```bash
# 生成 TF 树图
ros2 run tf2_tools view_frames
# 输出 frames.pdf

# 查询关键变换
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link laser_frame
```

确认：
- 每条边都存在，没有断链
- 数值与物理实测吻合
- RViz2 中 Add → TF 显示所有坐标轴，物理意义正确

### 闭环验证

跑 mapping 模式，让机器人闭合回到起点：

```bash
ros2 launch mentorpi_bringup mapping.launch.py
# 遥控走一个 3×3m 闭合路径回到起点
```

- 闭合误差 < 5%（直线漂移）→ Part 1 标定 OK
- yaw 闭合误差 < 5°→ Part 2 标定 OK
- RTAB-Map 点云无"分层"伪影 → Part 3 标定 OK
