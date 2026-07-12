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

- [x] Pi 上 `colcon build` + 重启 `remote.launch.py`。（2026-07-05）
- [x] 确认 IMU 真正进入 EKF：`tf2_echo base_link imu_link` 有输出、
      `/odometry/filtered` 45Hz、`/imu/data` 49Hz。（2026-07-05）
- [x] 运动原语 + 语音链路端到端：`robot_cli.py ws://raspberrypi5:9090
      rotate ±90`（gyro 闭环误差 0.2°）、`move forward/backward 0.3`
      （odom 计 0.291m，~3% 待轮径标定）、`stop` 服务。期间修掉 teleop
      零流争用 /cmd_vel bug + rosbridge 改为默认常驻。（2026-07-05）
- [x] slam_3d 带 lidar 融合**重新建图**：484 位姿、49 视觉回环 + 219 激光
      近邻链接（融合实战生效）、~15m 路径、160MB db。（2026-07-05）
- [x] loc_3d 重定位实测：重启后锁定 map→base_link=(−0.98, −1.03)。注意
      手搬机器人 = 绑架，需重启模式清 odom cache。（2026-07-05）

## 4. 高斯泼溅管线端到端

- [x] 导出：186 帧 + 种子点云（`gs_work/gs_dataset_full/`）。（2026-07-05）
- [x] 训练：**本机 RTX 4070 SUPER 直接可训**（venv 在
      `/media/luo/Game/venvs/nerfstudio`，NTFS 上可运行；ninja 需在 PATH；
      导出 checkpoint 需 `TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1`）→
      `gs_work/exports/splat/splat.ply`（47.3 万高斯）。（2026-07-05）
- [x] 查看端：`live_rerun.py --splat --cloud --serve` 双层显示，splat 与
      SLAM 点云对齐确认。web viewer URL 里 `+` 必须编码为 `%2B`。（2026-07-05）
- [x] **最终合体验证**：loc_3d 真实位姿锁定 (−0.80, −1.31) → Rerun 里
      实时机器人 + 视锥出现在 splat 场景,用户确认可用。(2026-07-11)
- [ ] **验证 foxglove 二进制通道**（代码完成+假服务器测试通过,未上真机）：
      `live_rerun.py --robot <ip>`（默认即 foxglove :8765）,对比流畅度;
      相机需直插 Pi 蓝口（挂 hub 会双流带宽塌陷 → 3fps）。
- [ ] 手机端 `--serve` 实测。
- [ ] 跟进 Rerun 原生 3DGS 渲染支持，落地后替换点云 fallback。
- [ ] 增量建图实测：同 db 再进 slam_3d + `load_all_nodes:=true`（CLI 直接
      launch,supervisor 的 SetMode 还没有该字段——要用再加）,确认新旧
      会话合并、GS 重训后场景更新。
- [ ] 低光提示：夜间/暗处彩色相机掉到 1-4Hz、特征枯竭 → 重定位静默失败
      （深度不受影响）。特征签名:color 低频+depth 正常。对策:开灯或
      白天作业;可选研究固定曝光参数。

## 4.6 定位精度提升计划（2026-07-11 定,按性价比排序）

现状:loc_3d 有明显偏移。误差 = 粗标外参(恒定偏差) + 薄图(锚点不准)
+ 修正间隙里程计漂移(2Hz 修正,间隙靠轮速+陀螺仪) 的叠加。

**精度天花板认知**(2026-07-12):栅格 0.05m + 激光噪声 ~2cm + 麦轮
打滑/舵机抖动 → 该平台合理上限约 **2~5cm / 1~2°**,到了就收工,再往上
是换传感器不是调参。

- [x] **第 0 步·判别偏移模式**(2026-07-12 完成):loc_check.json 布局
      观察 /scan vs /map 墙面。判别结果 = **匀速旋转漂移**(静止时位置
      比特级不变、yaw 0.25°/s 匀滑)→ 根因是陀螺仪零偏 +0.28°/s,EKF
      把它积成 ~15°/min 自转。已修:base_node 在线零偏估计(停车 EMA
      学习,发布前扣除)。修后用户确认"定位比较贴合"。
- [x] 里程计残余·gyro 零偏自估计:同上,已实现(`gyro_bias_estimation`
      参数,默认开)。麦轮横移打滑 ±5% 仍是修正间隙漂移的下限来源。
- [x] **AprilTag 相机外参精标**(2026-07-12 完成,scripts/
      calibrate_camera_extrinsic.py):平板全屏显示 tag36h11(黑框实测
      117.5mm),小车自动采集(弧站+测距规划直线站),位置-only 手眼求解,
      3 组数据中位数:光心 x=0.085(壳測 0.143)、y=0.023、yaw -2.4°、
      pitch +0.9°、roll -0.2°,已部署 base.launch.py。z=0.095 尺量固定
      (平面运动不可观)。踩坑记录见脚本 docstring 与 commit。
      **注意:现有 rtabmap.db 是旧外参建的,下次白天增量重扫后相机-激光
      一致性才完全兑现。**
- [ ] **白天增量续图增厚地图**:`slam_3d` + 同 db + `load_all_nodes:=true`
      再扫 5-10 分钟(多角度、多回环),锚点密度翻倍。注意 supervisor
      /mode/set 尚不透传 load_all_nodes(需 CLI 或加字段)。
- [ ] **rtabmap 修正加密/加宽容**:`Rtabmap/DetectionRate` 2→4Hz(看 Pi
      余量)、loc 模式下确认激光 ICP 修正(Reg/Strategy=1)真在生效
      (yaw 锁准靠它)、必要时 `Vis/MinInliers` 20→15。
- [ ] 远期:VIO(相机 IMU 重新启用做视觉惯性里程计)或 GS 渲染式定位
      (research 向,roadmap P4)。

## 4.7 传感器演进:pan/tilt 单目相机方案（2026-07-12 定调）

如果把 Gemini 2L 换成 2 自由度云台单目相机,**定位精度可以保住,但
主力必须换成激光雷达**——现在的精度里激光 ICP 本来就承担大头:

- **定位主力 → 2D 激光**(slam_toolbox loc / AMCL),室内同样 2~5cm,
  与相机完全解耦,精度不降。
- **失去的能力**:rtabmap RGB-D 定位/3D 彩色地图/GS 数据集导出
  (单目无度量深度,rtabmap 建图要求 depth 或双目)。想保 3D 地图,
  深度相机留着"建图会话专用",平时不跑。
- **动态外参问题**:云台一动 base_link→camera 就不是静态 TF。方案 =
  舵机角度实时发 joint TF;但 PWM 舵机精度 ~1-2°,3m 外即 5-10cm
  投影误差 → **参与 SLAM/定位的帧只在云台归中锁死时取**,自由转动
  时相机只做感知(找人/看物/VLA 输入),不进定位管线。
- **建议架构**:激光管定位、云台单目管感知,各干强项;VLA/语音的
  motion primitive 底座不受影响。

## 4.5 今日遗留问题（2026-07-05 晚，按优先级）

- [x] **验证 USB hub 供电拓扑**(2026-07-12)：相机挂 hub 5000M 链路 ✓;
      整个白天多轮驱动(VLA 任务、原语冲墙测试、推箱)后 dmesg
      over-current = 0 ✓。雷达是串口供电(不走 USB 5V)。
- [ ] **supervisor 开机死锁**（一次复现）：18:00 boot 实例发完 startup
      chime 后主线程 futex 死锁，服务调用永挂。当时带
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST（已撤销），可能相关。
      加锁超时日志/看门狗自愈；观察是否复发。
- [ ] **WiFi 掉线**（~6 次/天，power_save=off、信号 −37dBm、无欠压
      throttled=0x0、无驱动报错）：嫌疑=路由器踢客户端或 brcmfmac 软挂。
      判别：掉线时用手机 ping 192.168.8.117。**台架调试建议直接插网线**。
      DDS 抗断网方案：FastDDS interface whitelist XML（LOCALHOST 环境
      变量已证伪，会弄坏图像话题/action/supervisor）。
- [x] **相机 USB 卡死恢复路径**(2026-07-12 完成)：新增 `camera_watchdog`
      节点(mentorpi_bringup)托管 camera.launch.py,监测
      /camera/depth/camera_info,帧停发 >20s 或进程死亡 → SIGINT 驱动 →
      usbreset 2bc5:0670 → 重启,无限重试(~1min/轮)。单独 usbreset 无效
      的场景(需冷启)仍是残余风险,遇到再说。
- [ ] STM32 电源开关状态检查提示：/battery ≈4.4V + IMU 活 + 电机不转 =
      开关没开（见 docs/power_troubleshooting.md）。
- [x] **避障兜底的矮障碍盲区**(2026-07-12 当天补完)：
      depthimage_to_laserscan 取深度光轴 ±4° 高度带(距地 ~5-14cm)→
      `/depth_scan`(13Hz, 5.6% CPU),base_node 守卫并入前向扇区,
      低障停止线 0.45m(深度相机 0.2m 内无回波,须在还看得见时拦停)。
      深度流停更自动退化为纯激光。已实车验证拦停。侧/后方向仍只有
      激光(相机只朝前),VLA 提示词已告知模型自行避让矮障。
- [x] /guard/blocked 改 transient_local(初始发布 '',晚订阅立即可读)。

## 5. 部署 / 安全

- [ ] 远程接口目前局域网裸跑无认证；对外暴露前给 foxglove_bridge /
      rosbridge 加 nginx + TLS + basic auth（CLAUDE.md 安全模型一节）。
