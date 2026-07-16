# 实车标定交接文档

> 状态快照：2026-07-16（原 07-14 版已过时处就地更新）。此文档供下一个会话（Fable）冷启动接手实车标定。
> 分支 `feat/voice-vla-extensions`。标定总计划见 `docs/calibration.md`；本文件是**当前进度 + 下一步操作手册**。

## 一句话现状

TF 树标定分三部分（轮速里程计 → IMU → 相机）。**相机（Part 3）已完成**（2026-07-12 AprilTag 手眼标定，写回 xacro；但 07-14 螺丝重紧后需重标，见 §3.5）。**2026-07-16 已完成**：陀螺 z 符号确认（§1）+ 对墙旋转标定（§2，k_geom=1.117 → 有效轴距 0.1528/0.1575，k_gyro=1.0071 → `gyro_scale_z=0.9930`），详见 TODO.md Step 1.2 / 2.1 / 2.4。**剩余**：§3 方形闭环验收、§3.5 相机外参重标（白天）、Step 5 重扫（白天）、Step 1.3 UMBmark（可选，CCW/CW 差 2.1% 轻微不对称）。

## 硬件访问（已授权）

- Pi: `pi@192.168.8.117`，sudo 密码 `42834338`（用户已授权 Claude 直接跑 sudo）。
- 非交互 sudo：`ssh pi@192.168.8.117 'echo 42834338 | sudo -S <cmd>'`
- 官方 vendor SDK 在 `/home/pi/workdir/mentorpi/src/`，**只读参考，勿改**。
- workspace 在 Pi 上：`/home/pi/workdir/mentorpi/mentorpi_ws/`
- 系统 Python 必须用 `/usr/bin/python3.12`，**不要用 conda 的 3.13**（rclpy ABI 会崩）。
- 部署流程：dev 机 `git push` → Pi `git pull` + `colcon build --packages-select <pkg>` + `sudo systemctl restart mentorpi-remote`。

## 远程 CLI 的两个大坑（这次踩过，务必记住）

1. **conda 污染**：ssh 用 `bash -lc`（login shell）会自动 activate conda（Python 3.13），rclpy ABI 撞车。
   **对策**：用 `bash -c`（非 login）并显式 `export PATH=/usr/bin:/usr/local/bin:/opt/ros/jazzy/bin`，然后 `source /opt/ros/jazzy/setup.bash` + `source ~/workdir/mentorpi/mentorpi_ws/install/setup.bash`。验证 `echo $CONDA_PREFIX` 为空、`which python3`=`/usr/bin/python3`。
2. **服务用默认 RMW（FastDDS）+ ROS_DOMAIN_ID=0**（base_node environ 里没设任何 RMW/DOMAIN 变量）。CLI 别乱 export `RMW_IMPLEMENTATION`，保持默认即可匹配。

## 这次没电导致的假象（别再被骗）

关机前 `/imu/data_raw`、`/odom`、`/tf` echo **全空**，但 `ros2 action list` / `ros2 topic info`（元数据发现）**正常**。当时排查了 conda、RMW、python 块缓冲（`stdbuf -oL`），都不是根因。
**真因：电池欠压**。STM32 欠压时串口上报失效，base_node 发布器还在但收不到数据 → 所有 STM32 派生话题（imu/odom/battery）为空，EKF 无输入 → `/tf` 也停。对应 CLAUDE.md「lipo 欠压」已知问题。
**教训**：开工前先确认电量。`ros2 topic echo --once /battery` 看 voltage（满电 8.4V，≤6.6V 该充电；≈4.4V 是电源开关没开的寄生电压）。

## 已完成的改动（已写盘，**未 commit**）

`git status`：仅 `src/mentorpi_description/urdf/mecanum.xacro` 被修改（+11/-5），共 3 处：

- **`wheel_diameter`** 属性：`0.065` → **`0.0636`**（与 base_node 对齐，见文末「几何常量」）。
- **`laser_joint`** origin：`xyz="0 0 0.18"` → **`xyz="-0.014 0 0.18"`**
  依据：用户尺量激光雷达中心后偏 1.4cm、左右居中。z 已实测。RPY（安装 yaw）仍待墙面 line-fit 精化。
- **`imu_joint`** origin：`xyz="0 0 0.05"` → **`xyz="-0.054 0 0.05"`**
  依据：STM32 板中心后偏 5.4cm、左右对称。**注意：IMU 平移是装饰性的**——EKF 只融合 gyro vyaw、Madgwick 只用重力方向，二者都与 IMU 在刚体上的位置无关；只有轴向对齐（RPY）要紧。z 是估计值。
- 静态 IMU 已读：`ax≈+0.21, ay≈+0.29, az≈+9.56 m/s²` → **z 轴朝上、板基本水平、无轴翻转**。

改完 xacro 需在 Pi 上 `colcon build --packages-select mentorpi_description` + 重启 base 才生效。**当前 xacro 改动还没部署到 Pi**（关机前正在验证阶段）。

## 待办：充电后按顺序做

前置：小车充满电，摆到**正对一面平墙 1–1.5m**处，四周留 ≥0.5m 空档（原地旋转用，obstacle guard 不拦旋转）。开机确认服务 `systemctl is-active mentorpi-remote` = active，`/battery` voltage 健康。

### 1. ~~陀螺 z 轴符号确认~~（✅ 2026-07-16：CCW 时 gz 峰值 +0.64 rad/s，符号正确，无需改）
左转（CCW，正角速度）时 `/imu/data_raw` 的 `angular_velocity.z` 应 > 0。命令 CCW 小旋转并读 gz 符号：
```bash
ssh pi@192.168.8.117 'bash -c "
export PATH=/usr/bin:/usr/local/bin:/opt/ros/jazzy/bin
source /opt/ros/jazzy/setup.bash; source ~/workdir/mentorpi/mentorpi_ws/install/setup.bash
( stdbuf -oL timeout 6 ros2 topic echo --field angular_velocity.z /imu/data_raw >/tmp/gz.txt ) &
sleep 1
ros2 action send_goal /motion/primitive mentorpi_msgs/action/MotionPrimitive \"{type: rotate, distance: 0.5, max_speed: 0.5}\"
wait; sort -g /tmp/gz.txt | tail -1"'
```
gz 峰值为正 → 符号正确（左转=正），符合 REP-103，无需改。若为负 → IMU z 轴反装，需在 imu_joint RPY 加 `roll=pi`（或 base_node 里翻 gz 符号，二选一并记录）。

### 2. ~~Step 3 对墙旋转标定~~（✅ 2026-07-16 完成，结果见 TODO.md Step 1.2/2.4；注意脚本 totals 曾有双向相消 bug，已修——看每段/每方向数据）
脚本已写好并离线验证过（合成扫描 <0.03° 误差）：`scripts/calibrate_rotation.py`。**在 dev 机跑**（用 aire-venv，有 numpy/websockets），走 rosbridge :9090：
```bash
# dev 机，AIRE 的 venv
python scripts/calibrate_rotation.py ws://192.168.8.117:9090 --turns 2 --speed 0.5 --dir both
```
产出：
- **Step 1.2** `k_geom = odom/actual` → `wheelbase+track_width *= k_geom`（等比分配后改 `base_node.py` 的 `_mecanum` 常量，并同步 `mecanum.xacro` 的 `wheelbase`/`track_width`）。
- **Step 2.4** `k_gyro = ekf/actual` → 若 |k_gyro−1|>0.5% 则 gyro_z 需乘 `1/k_gyro`（base_node 需新增比例参数）。
- CCW/CW 对称性差 >2% 提示左右轮不对称（UMBmark）。
以激光墙面 line-fit 为 ground truth。若 rosbridge 拿不到 `/scan`，检查 lidar 和 QoS（脚本内有提示）。

### 3. Step 4 验收（🔶 2026-07-16 CCW 圈 PASS：估计误差 76mm/2.7% + 3.55°；CW 圈没电中断待补。工具换成 `scripts/acceptance_square.py`（激光 ICP 真值，零位精度 0.6mm/0.05°），odom_calib.py 不再需要）
3×3m 方形闭环，回到起点看 x/y/yaw 残差。可用 `scripts/odom_calib.py`（`/usr/bin/python3.12 scripts/odom_calib.py`，在 Pi 上跑，回车报告 dx/dy/dist/dyaw）辅助分段测量，或直接看闭环漂移。

### 3.5 ⚠️ 相机外参重标（2026-07-14 新增，白天做，先于 Step 5）
**事件**：相机螺丝松动、用户已重新拧紧，**俯仰角可能变了** → 2026-07-12 的外参失效，必须重标。
**没有"只标 pitch"捷径**——AprilTag 手眼法一次性重解整个 `base_link→camera_link`（z 尺量固定）。
```bash
# dev 机，aire-venv 有 pupil_apriltags/scipy/websockets；idle 模式即可，obstacle guard 全程有效
python scripts/calibrate_camera_extrinsic.py ws://192.168.8.117:9090 \
    --tag-size 0.1175 --update-xacro
```
平板显示 `tag36h11 #0` 贴墙立着（倾斜没关系），小车摆到 tag 大致在视野内，**需充足光照**。
`--update-xacro` 原子写回 `camera_joint`（`--max-update-rms-mm` 默认 20mm 门槛拦离谱结果）。
生效：Pi 上 `colcon build --packages-select mentorpi_description` + `sudo systemctl restart mentorpi-remote`。
**顺序**：先此步重标外参 → 再 Step 5 重扫，否则重扫用旧外参。

### 4. Step 5（白天再做）
用**重标后的**相机外参增量重扫建图。低光会掉相机帧，留到白天，紧跟 §3.5 之后。

## 关键文件索引

| 文件 | 作用 |
|------|------|
| `src/mentorpi_description/urdf/mecanum.xacro` | 固定 TF 源头（本次改了 laser/imu joint，未 commit） |
| `scripts/calibrate_rotation.py` | Step 1.2 + 2.4 对墙旋转标定（已 commit b19ecf6） |
| `scripts/odom_calib.py` | 纯 /odom 位移/转角读数辅助 |
| `scripts/calibrate_camera_extrinsic.py` | 相机 AprilTag 手眼标定 + `--update-xacro` 写回（Part 3 已用） |
| `base.launch.py` (mentorpi_bringup) | 常驻硬件，含 robot_state_publisher(渲染 xacro runtime_mode:=true) + joint_state_publisher(轮子零位) |
| `src/mentorpi_bringup/config/ekf.yaml` | EKF：odom0 只融 vx/vy，imu0 只融 vyaw |
| `TODO.md` | 标定各步勾选状态 |
| `docs/calibration.md` | 标定总计划（三部分） |
| `memory/calibration-state.md` | 跨会话记忆 |

## base_node 相关常量（改几何后要同步的地方）

- `mentorpi_base/base_node.py` 的几何：`wheelbase=0.1528`、`track_width=0.1575`（`_mecanum` 内硬编码，**有效几何**，2026-07-16 k_geom=1.117 标定；物理尺量 0.1368/0.1410）、`wheel_diameter=0.0636`（declare_parameter 默认值，Step 1.1 已标定）、`gyro_scale_z=0.9930`（Step 2.4，declare_parameter，热更新）。
- `mecanum.xacro` 顶部 `wheelbase`/`track_width`/`wheel_diameter` 属性必须与 base_node 一致（已同步 0.1528/0.1575/0.0636；轮子视觉位置比实物外扩 ~8mm 是已知取舍）。
