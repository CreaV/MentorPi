# 纯视觉建图极限探索 Roadmap（2026-10-03 草案）

目标：在**不依赖激光雷达**的前提下，摸清 MentorPi（Gemini 2 + Pi 5，局域网 4070S 可用）
的 3D 建图能力上限，并把有效改进落回现有系统。雷达保留，但角色从"建图传感器"
变为"**评测真值**"和"安全兜底"。

> 本文是规划，不是已实施状态。所有"预计/可能"项都需按各阶段的验收标准实测。

## 0. 调研结论：那些 OAK 极限演示是怎么做到的

调研对象：B 站"OAK 相机当流星锤甩"和"OAK 定位迪士尼最速过山车"两条视频（视频页
与评论区在当前网络环境无法访问，以下来自搜索到的厂商文章、官方文档和开源仓库）。

| 来源 | 关键信息 |
|---|---|
| OAK China / 因子空间 Factor Perception SDK | 过山车演示用 OAK-D-Pro-W。**前端**：HF-Net 在相机端 NPU 上跑（局部特征点 + 全局描述子，约 20Hz），配合 200Hz IMU 做紧耦合 VIO；**后端：RTAB-Map**；回环用 HF-Net 全局描述子做视觉位置识别（VPR） |
| Luxonis 官方 DepthAI v3 示例 | Basalt VIO / RTAB-Map VIO + **RTAB-Map SLAM**；双目 640×400、30–60fps、IMU 100–200Hz |
| `Factor-Robotics/depthai-hfnet` | HF-Net 的 OAK 端部署，已作为 SLAM 系统的一部分接进 RTAB-Map |
| RTAB-Map 本身 | 已支持 SuperPoint（`Kp/DetectorStrategy=11`，需 torch 编译）+ SuperGlue（`Vis/CorNNType=6`）；Labbé 2022 多会话光照不变重定位（同一地点不同光照多次建图） |
| VIW-Fusion / VINS-on-Wheels | 地面车做平面运动时 VIO 有额外不可观自由度（尺度等），融合轮速可恢复；光照剧变场景下 VIWO 明显优于纯 VIO |
| MASt3R-SLAM（CVPR 2025） | 单目稠密 SLAM，单张消费级 GPU 约 15 FPS，精度和鲁棒性超过 ORB-SLAM / DROID-SLAM |

**核心结论**：极限演示的配方 = **强视觉惯性前端**（学习特征 + 紧耦合 IMU + 时间同步 +
全局快门/广角）+ **RTAB-Map 后端**（学习全局描述子做回环）。

**我们和它的差距在前端，不在后端**：
- 后端都是 RTAB-Map，仓库里的多会话、supervisor 托管、GS 导出这些基础设施都能直接复用。
- 前端是本 roadmap 的主攻方向。现在的前端是"cmd_vel 积分 + STM32 陀螺仪"的 EKF，完全没有视觉里程计；相机 IMU 关着；STM32 IMU 和图像没有时间同步。

两条视频证明的是**跟踪不丢**（短时、剧烈运动），不是长距离地图一致性；后者要靠后端回环。

## 1. 现状与约束

| 项 | 现状 | 对视觉建图的影响 |
|---|---|---|
| 相机 | Gemini 2，彩色 640×480@15 MJPG，深度 640×400@15，D2C 硬件对齐；卷帘快门彩色 | 车速低，卷帘影响小；15fps 偏低，限制快速旋转 |
| 相机 IMU | `enable_accel/gyro: false` | 紧耦合 VIO 的前提缺失 |
| 里程计 | 无编码器，cmd_vel 积分 + 陀螺仪 | 尺度和打滑无法自校 |
| 后端 | RTAB-Map 2Hz，`Force3DoF`，融合 `/scan` | 强制平面，坡道和多层不可用 |
| Pi 5 | 3D 模式核心 CPU 约 75% | 新增前端要么很轻，要么卸载 |
| 4070S | 同局域网，已用于 AIRE / GS 训练 | 学习特征、稠密 SLAM、重型后端的主力 |
| 时钟 | Pi 无 RTC，NTP 会跳变（已有 clock-jump-guard） | 跨机计算必须先解决亚毫秒级时间同步 |
| 录包 | `scripts/record_3d_bag.sh`（MCAP）已有 | 离线评测的基础，需要扩充话题 |

## 2. 算力分层（目标架构）

```
Pi 5（实时 / 安全 / 不依赖网络）            4070S（重计算 / 可断网）
─────────────────────────────            ───────────────────────────────
Gemini 2 驱动 (+IMU, +IR 可选)            rtabmap 后端（SuperPoint/SuperGlue,
base_node / obstacle_guard / motion          全局描述子回环, 不再 Force3DoF）
EKF：轮速 + 陀螺仪 (+ VIO, 可选)   ──RGB-D 压缩 2–5Hz + odom/TF──▶
轻量 VIO（若 Pi 跑得动）                  学习型 VIO / 稠密 SLAM（离线优先）
rgbd_sync + 压缩                  ◀──── map→odom TF（低频）──────
                                          3DGS 训练、Rerun 可视化
```

原则：
1. **控制和安全回路永远不跨 WiFi**。Pi 上的 EKF 在 odom 系下照常工作，4070S 只发低频的 `map→odom`。断网时只是地图暂停更新，车不会失控。
2. **VIO 是 EKF 的额外输入，不替代 EKF**。VIO 跟丢时，EKF 自动退回轮速 + 陀螺仪，保持现在约 20ms 的延迟和鲁棒性。
3. **先离线、后实时**。同一份 bag 能反复比较多种算法，确认真有收益再做实时化。

## 3. 分阶段计划

### P0 — 修现有链路（只动 Pi，约 1–2 天）

来自 2026-10-03 代码审查，与视觉无关，但不修的话后面的评测数据不可信：

- [ ] supervisor 启动 3D 模式时传 `sigterm_timeout` / `sigkill_timeout`。现在 `ros2 launch` 默认约 10s 就 SIGKILL，90s 宽限形同虚设；clock-jump-guard 的重启同样依赖这个宽限。
- [ ] `Grid/MaxGroundHeight` 改为相对 `base_link` 的值（`base_link` 离地 0.0505m），或者运行态补一个 `base_footprint`。
- [ ] 核实相机 pitch（-0.1196 rad = 抬头约 6.9°）；低障虚拟激光改成在 `base_link` 系按高度切片。
- [ ] RGB-D 同步收紧：驱动开帧同步，加 `rgbd_sync`，`approx_sync_max_interval` 约 0.03s。
- [ ] rtabmap 参数抽到共享 YAML；加 `Rtabmap/TimeThr`。

**验收**：连续 3 次切换模式不损坏数据库；栅格地图上 5–10cm 的低障碍可见。

### P1 — 评测基础设施（"没有尺子就谈不上极限"，约 1 周）

- [ ] `camera.launch.py` 增加可选开关 `enable_imu` / `enable_ir`，默认关，不影响现网。
  - 先实测 Gemini 2 的 IMU 时间戳和图像是否同一时钟、IMU 频率能开到多少。
  - 驱动能输出几路 IR、能否开关投射器，都要先查驱动参数再定方案。
- [ ] 用 Kalibr 做相机-IMU 标定（外参 + 时间偏移）：用 AprilGrid 标定板，录一段 60–90s 的充分激励数据。
- [ ] Pi ↔ 4070S 时间同步：chrony 以局域网主机为源，验收指标是偏差 < 1ms。也让 clock-jump-guard 少触发。
- [ ] 扩充 `record_3d_bag.sh`：加 IMU、IR，以及 `/tf` 原样记录（已有）。
- [ ] **刁难场景数据集**，每段 1–3 分钟，全部带雷达：
  1. 快速原地旋转
  2. 关灯（只有 IR）
  3. 白墙或低纹理
  4. 长直走廊（模拟隧道）
  5. 坡道
  6. 大回环（测回环）
  7. 多房间往返
  8. 同一路线白天和夜晚各一次（测光照不变重定位）
- [ ] 真值：离线用雷达跑 slam_toolbox 或 rtabmap ICP 的轨迹作参考；短段再加 AprilTag。
- [ ] 评测脚本：用 `evo` 算 ATE / RPE，另统计跟踪丢失率、回环成功率和误回环数，输出一张"场景 × 方案"表。

**验收**：现有 RTAB-Map + EKF 在 8 个场景上有基线分数。

### P2 — 前端升级：视觉惯性里程计（离线在 4070S 上对比，约 2–3 周）

候选方案（按改动由小到大）：

| 方案 | 特点 | 预期价值 |
|---|---|---|
| RTAB-Map `rgbd_odometry`（F2M）+ IMU 作初值 | 零新依赖 | 最快出结果，作对照组 |
| ORB-SLAM3 RGB-D-Inertial | 成熟，社区有 Gemini 2 实践 | 紧耦合 VIO 基线 |
| OpenVINS / Basalt | 轻量滤波 / 优化 VIO | 最可能在 Pi 5 上实时跑 |
| VIW-Fusion 类（视觉 + IMU + 轮速） | 专治地面车尺度不可观 | 理论上最适合本车；轮速是 cmd_vel 而不是编码器，收益需实测 |
| DROID-SLAM / DPVO | 学习型，GPU | 低纹理、模糊场景的上限参考 |

- [ ] 每个方案跑完整个 P1 数据集，出对比表。
- [ ] 选出的方案作为 EKF 的 `odom1` 接入：只融合 vx/vy/vyaw，协方差动态给，跟丢时拉大。
- [ ] 实测 Pi 5 能否实时跑（CPU、延迟）；跑不动就放在 4070S 上，只服务建图，不服务控制。

**验收**：在长走廊、白墙、快速旋转三个场景，漂移和丢失率比基线明显下降，阈值在 P1 出基线后再定。

### P3 — 后端上 4070S：学习特征和学习回环（约 2 周）

- [ ] Pi 端：`rgbd_sync` + 压缩，降到 2–5Hz（rtabmap 检测频率本来就是 2Hz），带宽目标 < 2 MB/s。
- [ ] 4070S 端：rtabmap 以支持 torch 的方式编译。
  - 局部特征：`Kp/DetectorStrategy=11`（SuperPoint）+ `Vis/CorNNType=6`（SuperGlue）。
  - 回环：加全局描述子（NetVLAD 一类，rtabmap 的 Python 描述子接口，需验证编译选项）。
- [ ] 解除 `Reg/Force3DoF`：EKF 关掉 `two_d_mode`，融合 Madgwick 的 roll/pitch，用于坡道（配合 P2 的 VIO 提供 z）。
- [ ] 多会话光照不变：同一场所在不同光照下各建一次，合并成一张图（Labbé 2022 的做法，仓库已支持续图）。
- [ ] supervisor 增加 `slam_3d_remote` 模式：Pi 只起同步和压缩，后端在 4070S 上，`map→odom` 回传。

**验收**：P1 的昼夜重定位场景成功率提升；误回环数不增加。

### P4 — 暗光和极端场景（约 1–2 周，部分需要硬件）

- [ ] 暗光跟踪改用 IR 流。投射器点阵会骗特征跟踪，要么关投射器加外置 850/940nm 补光灯，要么做逐帧交替（看驱动是否支持）。
- [ ] 外置红外补光：先估成本和功耗，注意 USB 供电已经很紧张，见 `power_troubleshooting.md`。
- [ ] 坡道：P3 解除 3DoF 后跑 P1 的坡道数据，看 3D 地图是否还会扭曲。
- [ ] 隧道和长走廊：ICP 用点到面 + `Icp/PointToPlaneMinComplexity` 拒绝退化修正；回环阈值收紧。

### P5 — 去雷达实验与结论（约 1 周）

- [ ] 全链路关闭 `subscribe_scan`，用纯视觉方案重跑全部场景，和"视觉 + 雷达"对比。
- [ ] 稠密几何和外观对比：MASt3R-SLAM 离线重建、现有 3DGS 管线、RTAB-Map 点云三者对比。
- [ ] 结论文档：每个场景"纯视觉够用 / 需要雷达 / 都不行"，以及推荐配置。

## 4. 风险与未知

| 风险 | 缓解 |
|---|---|
| Gemini 2 的 IMU 和图像不是硬件同步，或时间偏移不稳定 | P1 第一件事就测；不行就退到松耦合（VIO 不融 IMU，只靠 EKF） |
| 15fps 彩色对快速旋转不够 | 测 30fps 下 Pi 的 CPU 和 USB 余量；或者 VIO 改用 IR 流 |
| WiFi 带宽和延迟抖动 | 后端只需要 2–5Hz；断网时本地 EKF 兜底 |
| 无编码器，VIWO 收益有限 | 考虑后续加编码器或光流传感器（硬件项，单独评估） |
| rtabmap torch 编译复杂 | 用 Docker 隔离在 4070S 上 |
| 长直、重复纹理的隧道 | 纯视觉的物理极限，P5 如实记录，不强求 |

## 5. 下一步

先做 P0（稳定现网），同时开 P1 的第一项：`camera.launch.py` 加 `enable_imu` 开关，并实测 Gemini 2 的 IMU 时间戳。

## 参考

- OAK China：VSLAM 新方案（Factor-VIO / HF-Net / RTAB-Map）— https://blog.csdn.net/oakchina/article/details/144296543
- OAK China：OAK D + 因子空间 Factor Perception SDK — https://www.oakchina.cn/product/oak-d-series-vslam%E5%9B%A0%E5%AD%90%E7%A9%BA%E9%97%B4%E6%84%9F%E7%9F%A5sdk-license/
- Luxonis：VIO/SLAM library evaluation — https://discuss.luxonis.com/blog/5952-vioslam-library-evaluation-for-oak-d
- Luxonis：Basalt VIO + RTAB-Map 示例 — https://docs.luxonis.com/software-v3/depthai/examples/rvc2/vslam/basalt_vio_rtab
- Factor-Robotics/depthai-hfnet — https://github.com/Factor-Robotics/depthai-hfnet
- Roller Coaster SLAM Dataset — https://opencv.org/roller-coaster-slam-dataset/
- RTAB-Map SuperPoint — https://github.com/introlab/rtabmap/blob/master/corelib/src/python/rtabmap_superpoint.py
- Labbé & Michaud 2022, Multi-Session Visual SLAM for Illumination-Invariant Re-Localization — https://arxiv.org/abs/2103.03827
- RTAB-Map 远程建图（rgbd_sync + 压缩）— http://wiki.ros.org/rtabmap_ros/TutorialsOldInterface/RemoteMapping
- VIW-Fusion — https://github.com/TouchDeeper/VIW-Fusion
- MASt3R-SLAM — https://arxiv.org/abs/2412.12392
