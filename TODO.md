# TODO

未完工事项汇总（2026-07-03 整理）。标定三部分的完整操作步骤在
`docs/calibration.md`，此处给出方法要点 + 验收标准。

## 1. 标定（docs/calibration.md，未开始，必须按 Part 1 → 2 → 3 顺序）

### Part 1 — 轮速里程计（先做，否则相机外参会吸收里程计误差）

- [ ] **Step 1.1 轮径 `wheel_diameter`**：地贴 5m 卷尺，`/usr/bin/python3.12
      scripts/odom_calib.py` 锁原点 → 手柄匀速直行 ~3m 停稳 → Enter 读
      `dist=d`，卷尺量实际 `D`，`wheel_diameter_new = old × D / d`。
      正反向各 3 次取平均。
- [ ] **Step 1.2 轴距和 `wheelbase + track_width`**：轮径标完后做。
      `timeout 5.24 ros2 topic pub -r 20 /cmd_vel geometry_msgs/Twist
      '{angular: {z: 0.3}}'`（理论 90°），odom_calib 读 `dyaw=ω`，激光投点/
      量角器量实际 `θ`，`(W+T)_new = (W+T)_old × ω / θ`。
- [ ] **Step 1.3 UMBmark**：4×4m 正方形顺/逆各 5 圈，判断 track_width 偏差
      与左右轮不对称（判据表见 calibration.md）。
- [ ] （视结果）横移 vy 若有系统性尺度误差，给 base_node 加 vy 尺度参数
      （麦轮横移打滑是系统性的，纯协方差盖不住尺度错）。
- [ ] **验收**：3×3m 闭合路径，直线漂移 < 5%。

### Part 2 — STM32 IMU

- [ ] **Step 2.1 轴向检查**：车静止水平，`ros2 topic echo /imu/data_raw
      --field linear_acceleration` 应见 `az≈+9.81, ax/ay≈0`；向前推车
      `ax` 应为正。异常按 calibration.md 判据表修 `base_to_imu` 的 RPY。
- [ ] **Step 2.2 精化 `base_link → imu_link` TF**：TF 已在
      `base.launch.py`（z=0.05、零旋转为估计值），按 2.1 结果改 RPY，
      平移尺子量。
- [ ] **Step 2.3 Gyro 零偏**：绝对静止 60s 记录 `angular_velocity` 均值
      （典型 0.001~0.01 rad/s）。长期方案见下面代码项（启动自估计）。
- [ ] **Step 2.4 Gyro_z 比例**：原地转实测 360°（物理量角），积分 gyro.z
      应 = 2π，不等则在 base_node 乘比例系数。
- [ ] **验收**：闭合路径 yaw 闭合误差 < 5°。

### Part 3 — 相机外参 `base_link → camera_link`（依赖 Part 1+2）

- [ ] **粗标（先做，高斯泼溅重建图之前）**：尺子量相机光心相对 base_link
      的 x/z 改 `base.launch.py`；pitch 用点云判据调——rtabmap 建图后在
      RViz/Rerun 里看地面是否水平（地面点 z 应 ≈0 且不倾斜），低头角按
      弧度填 pitch（10°≈0.1745）。
- [ ] **精标（AprilTag 法）**：打印 36h11 tag（边长 ≥10cm）平贴地面，
      `apriltag_ros` 输出 `T_camOptical_tag`，尺子量 `T_base_tag`，
      反推 `T_base_camLink = T_base_tag × T_camOptical_tag⁻¹ ×
      T_optical_camLink`。多帧取平均。依赖：`sudo apt install
      ros-jazzy-apriltag ros-jazzy-apriltag-ros`。
- [ ] 自研标定工具（暂定 `mentorpi_calibration` 包）：自动化上述流程，
      输出 static_transform_publisher 6 参数。
- [ ] **验收**：rtabmap 点云无"分层"伪影，地面厚度 < 2cm。

### 激光雷达 yaw 对齐（顺手项）

- [ ] 车正对平墙，RViz 里 `/scan` 墙面线应与相机点云墙面重合、且垂直于
      x 轴；有偏差改 `base_to_laser` 的 yaw。z=0.18 已实测无需动。

## 2. 代码待做

- [ ] **base_node 启动 gyro 零偏自估计**：上电静止 ~5s 取 gx/gy/gz 均值，
      之后发布前扣除。动机：EKF 现在直接融合 gyro vyaw，零偏会 1:1 积分成
      yaw 漂移（0.005 rad/s ≈ 17°/min）。需处理"启动时车在动"的场景
      （方差超阈值则跳过估计并告警）。
- [ ] **supervisor 停 3D 模式的宽限期**：`_stop_active` 目前 SIGINT 10s →
      SIGTERM 5s → SIGKILL。rtabmap 收尾要把内存图写回 db，大 db + SD 卡
      慢时 10s 不够，SIGKILL 配合 `Synchronous=OFF + journal=MEMORY` 是
      db 损坏（`addWordRef() Not found word`）的头号嫌疑。改为：slam_3d/
      loc_3d 模式 SIGINT 宽限 ≥30s，且 rtabmap 永不 SIGKILL（超时只告警）。
- [ ] `accel_limit_linear/angular`（base_node 新参数，默认 1.5 / 10.0）
      实车调参：手柄阶跃指令下对比 `/odom` 与实际位移，斜坡太缓/太陡都改。
- [ ] 地图保存进 supervisor/web UI（目前 2D 靠手动调 serialize_map 服务，
      手机端没有入口）。
- [ ] Foxglove layout（`mentorpi.json`）补 loc_3d 模式按钮（web SPA 已有）。

## 3. 本次改动的实车验证（代码已完成，未上车）

- [ ] Pi 上 `colcon build` + 重启 `remote.launch.py`。
- [ ] 确认 IMU 真正进入 EKF：`ros2 run tf2_ros tf2_echo base_link imu_link`
      有输出、`/odometry/filtered` 的 yaw 在原地转动时跟手（此前缺 TF 时
      robot_localization 静默丢弃 IMU）。
- [ ] slam_3d 带 lidar 融合**重新建图**（旧 rtabmap.db 里没有 scan 数据，
      NeighborLinkRefining 对旧图不生效），对比建图质量。
- [ ] loc_3d 重定位实测：重启 → 手机点 Loc 3D 选 .db → 移动到有纹理区域
      → 确认 `map→odom` TF 出现、位姿正确。

## 4. 高斯泼溅管线端到端（工具已完成并本地测过导出，未跑训练）

- [ ] 新图导出：`python3 scripts/export_gs_dataset.py ~/rtabmap_maps/xxx.db
      --output-dir ~/gs_dataset`，拷到训练服务器。
- [ ] 服务器 `ns-train splatfacto`（**必须** `--orientation-method none
      --center-method none --auto-scale-poses False`）→ `ns-export
      gaussian-splat` 得 splat.ply。
- [ ] 对齐自检：`live_rerun.py --splat splat.ply --cloud rtabmap_cloud.ply`
      两者应完全重合；不重合 = 训练时自动重定向没关。
- [ ] 手机端 `--serve` 实测（同一局域网浏览器打开打印的 URL）。
- [ ] 跟进 Rerun 原生 3DGS 渲染支持，落地后替换 live_rerun.py 的点云
      fallback（实体路径不变，只换 log 调用）。

## 5. 部署 / 安全

- [ ] 远程接口目前局域网裸跑无认证；对外暴露前给 foxglove_bridge /
      rosbridge 加 nginx + TLS + basic auth（CLAUDE.md 安全模型一节）。
