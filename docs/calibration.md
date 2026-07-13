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
| `base_link → imu_link` | xacro + robot_state_publisher | **是** — 平移/RPY 是估计值，需按 Part 2 精化 |
| `base_link → camera_link` | xacro + robot_state_publisher | 2026-07-12 已完成 AprilTag 手眼标定；z 为尺量固定值 |
| `base_link → laser_frame` | xacro + robot_state_publisher | z=0.18 已实测；x/y/RPY 待精量 |
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

### 工具：`scripts/odom_calib.py`

为了避免每次手动从 `ros2 topic echo` 里读数，仓库自带了一个小工具，订阅 **`/odom` topic**（base_node 直接发的纯 dead-reckoning，**不经过 EKF**，因此不被 IMU 污染），按 Enter 报告自上次原点至今的位移和转角，并自动重置原点。

```bash
# 终端 A: 起机器人 (任意 launch 都行，只要 base_node 在跑)
ros2 launch mentorpi_bringup mentorpi.launch.py

# 终端 B: 起标定工具 (注意必须用系统 Python 3.12，不能用 conda)
/usr/bin/python3.12 scripts/odom_calib.py
```

输出示例：
```
[ready] origin locked: x=+0.0012 y=-0.0003 yaw=+0.05°
[01] dx=+2.987m  dy=+0.014m  dist=2.987m  dyaw=+0.412°  (raw rad=+0.00719)
```

交互：`<Enter>` 报告并重置原点；`r<Enter>` 只重置不报告；`q<Enter>` 退出。

### Step 1.1 — 直行标定 `wheel_diameter`

地上贴 5 米卷尺，机器人正向匀速跑 3 米。

操作：
1. 把车后轮贴在卷尺 0 米处对齐
2. 终端 B `odom_calib.py` 起来后会显示 `[ready] origin locked`
3. **按一次 Enter** 把当前位置锁为原点（消除工具启动时的微小漂移）
4. 手柄推车直行 3 米左右，**停稳**
5. 再按一次 Enter，工具报告 `dist = d`
6. 卷尺量实际值 `D`
7. `wheel_diameter_new = wheel_diameter_old × D / d`

重复 3 次取平均，反向再跑 3 次确认对称性。

### Step 1.2 — 原地旋转标定 `wheelbase + track_width`

> ⚠️ 必须先做完 Step 1.1。轮径未标准前转角误差会同时受 `wheel_diameter` 和 `wheelbase+track_width` 影响，无法单独区分。

mecanum 车原地转向时，`wheelbase + track_width` 的和决定角速度比例（见 base_node.py:285 的 `vp = wz * (wheelbase + track_width) / 2.0`）。

操作（推荐用单段 90°，避开 ±180° 归一化问题）：
1. 地上贴参考线，机器人对齐
2. 发指令 `wz = 0.3 rad/s` 持续约 5.24 秒（理论转 90°）：
   ```bash
   timeout 5.24 ros2 topic pub -r 20 /cmd_vel geometry_msgs/Twist '{angular: {z: 0.3}}'
   ```
3. 标定工具按 Enter 读 `dyaw` —— odom 报告的转角（记为 `ω`，本质上等于命令积分 `wz × t`）
4. 用激光在墙上投点 / 手机量角器 app 量物理实际转角（记为 `θ`）
5. `(wheelbase + track_width)_new = (wheelbase + track_width)_old × ω / θ`

推导：
```
base_node 输出的 wheel_speed = wz_cmd × (W+T)_old / 2
物理 ω_real = 2 × wheel_speed / (W+T)_phys = wz_cmd × (W+T)_old / (W+T)_phys
→ θ_real = ω_odom × (W+T)_old / (W+T)_phys
→ (W+T)_phys = (W+T)_old × ω_odom / θ_real
```

> `odom_calib.py` 的 `dyaw` 归一化到 ±180°。要转大角度做平均，分段按 Enter 累加各段 dyaw。

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

> `base_link → imu_link` 已由 xacro + robot_state_publisher 发布。当前数值仍是估计值（z=0.05、零旋转），本 Part 用于验证轴向并精化 RPY。

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

### Step 2.2 — 精化 `base_link → imu_link` 静态 TF

TF 现在由 `mentorpi_description/urdf/mecanum.xacro` 的 `imu_joint` 经 robot_state_publisher 发布。按 Step 2.1 的判断修改其 RPY；平移用尺子量控制板位置填入（平移对 gyro/orientation 融合几乎无影响，轴向对齐才关键）。

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

相机外参保存在 `src/mentorpi_description/urdf/mecanum.xacro` 的 `camera_joint` 中。2026-07-12 已完成 AprilTag 手眼标定，当前值为 `xyz=0.0846 0.0231 0.095`、`rpy=-0.0032 0.0149 -0.0422`；其中 z 是尺量固定值，位置-only 残差约 9mm。

**方法选择：AprilTag（精标方法 B）+ 后续自研标定工具**

### 标定结果写回 URDF

默认求解只打印 `camera_joint`，不修改文件。确认数据质量后可显式写回：

```bash
python scripts/calibrate_camera_extrinsic.py --solve-only /tmp/calib_run \
  --update-xacro --max-update-rms-mm 20
colcon build --packages-select mentorpi_description mentorpi_bringup
source install/setup.bash
xacro src/mentorpi_description/urdf/mentorpi.xacro runtime_mode:=true \
  > /tmp/mentorpi_runtime.urdf
check_urdf /tmp/mentorpi_runtime.urdf
```

脚本只替换 `mecanum.xacro` 中唯一的 `camera_joint/origin`，写入前验证
XML，并使用临时文件原子替换。若位置 RMS 超过阈值、目标 joint 不唯一或 XML
无效，会拒绝修改。相机高度仍由 `--cam-z` 提供，写回前必须确认尺量值。

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
- 输出 camera_joint 的 xyz/rpy 参数
- 可选：直接更新 `mecanum.xacro` 的 `camera_joint`

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
