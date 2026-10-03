# 纯视觉建图极限探索 Roadmap（2026-10-03，修订版）

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

## 3. 目标与成功标准（2026-10-03 修订）

"预期"分三档，各自的判定标准不同。每完成一个阶段，都要回到这张表核对：

| 档位 | 内容 | 判定标准 | 预估可达性 |
|---|---|---|---|
| A. 能力边界结论 | 回答"不用雷达能做到什么程度" | 8 类场景 × 各方案的误差表，每个场景给出明确结论 | 高。快速验证 S 加上 P5 一定能产出 |
| B. 室内纯视觉可用 | 平地多房间、快速旋转、光照变化下，不用雷达也能建出可用地图 | 和雷达真值相比，平移 ATE < 行进距离的 2%，回环无误合并，跟踪丢失率 < 1% | 中高 |
| C. 极端场景 | 暗光、坡道、长隧道 | 分场景定 | 暗光、坡道：有条件；长隧道、管道：低（纯视觉的物理极限） |

Gemini 2 和 OAK-D Pro W 的硬件差距（卷帘彩色、15fps、视场角约 90°、没有板载 NPU、IMU 同步未知、无编码器）是 C 档的天花板。计划不承诺复现 OAK 的极限演示。

## 4. 分阶段计划（修订：先快速验证，再按结果分支）

原计划按顺序写了 P0–P5，问题是关键信息要到 P2 才出现。修订后的顺序：

```
P0 修现网 ──┐
            ├─▶ S 快速验证（2 周）──▶ 决策点 ─┬─▶ P2 VIO 前端
S0 IMU 实测 ┘                                ├─▶ P3 后端上 4070S
                                             └─▶ 止步：瓶颈在相机硬件，出结论
                         P4 / P5 只在前面有正收益时才做
```

### P0 — 修现有链路（只动 Pi）

来自 2026-10-03 代码审查。

**代码已改（分支 `feat/3d-mapping-p0`），尚未在 Pi 上验证**：
- [x] rtabmap 节点设 `sigterm_timeout=85` / `sigkill_timeout=15`（作为 Node 参数直接传入，不依赖 launch 参数作用域），与 supervisor 的 90s 宽限对齐。
- [x] `Grid/MaxGroundHeight` 改为相对 `base_link` 的值：0.04 − 0.0505 = -0.0105，即离地约 4cm。
- [x] 新增 `rgbd_sync` 节点，先把 RGB-D 打包再交给 rtabmap，`approx_sync_max_interval=0.034`（半帧）。rtabmap 改为 `subscribe_rgbd` + scan。等 S0 测出真实偏移后再收紧。
- [x] rtabmap 参数抽到 `mentorpi_bringup/rtabmap_params.py`（Python 模块，可被两个 launch 导入），加单元测试，检查全部为字符串、两个模式只有预期的差异项。
- [x] 修正 `Reg/Strategy` 的注释；`rtabmap_mapping.launch.py` 透传 `load_all_nodes`。

**需要在 Pi 上做**：
- [ ] 连续 3 次切换 `slam_3d` → `idle`，看日志确认 rtabmap 正常退出（没有被 SIGKILL）、数据库能重新加载。
- [ ] 栅格地图：平地上没有地板误判成的假障碍，5–10cm 的纸箱能看到。
- [ ] `/rtabmap/info` 显示每帧都在处理（rgbd_sync 有输出）。
- [ ] 核实相机 pitch（-0.1196 rad = 抬头约 6.9°）：平地点云的地面是否水平。确认后再决定 depth_low_scan 是否改成在 `base_link` 系做高度切片。

**推迟到 S 阶段**：`Rtabmap/TimeThr` 要等测出 Pi 上 rtabmap 的实际处理耗时再设，盲设会让工作内存被过度清空。

### S0 — 相机 IMU 实测（第一个决策点，约 1 天）

**代码已改**：
- `remote.launch.py` / `base.launch.py` 新增 `camera_imu:=true|false` 参数（默认 false，现网行为不变），经 camera_watchdog 传给 `camera.launch.py` 的 `enable_imu`。
- 新增 `scripts/check_camera_imu_sync.py`。

**需要在 Pi 上测**：
- [ ] 开 `camera_imu:=true` 后，记录 Pi 的 CPU 和 USB 是否稳定，相机有没有掉线。
- [ ] 跑 `check_camera_imu_sync.py` 并量出：
  - IMU 频率和抖动；
  - IMU 与图像各自的"时间戳 − 接收时间"，判断是否同一时钟；
  - 彩色与深度的时间戳偏移（用于收紧 rgbd_sync）。

**决策**：
- IMU 时间戳和图像同钟、抖动小于 1ms，走紧耦合 VIO 路线（P2 优先）。
- 不同钟或抖动很大，只能做松耦合。这时重点转向学习特征和后端（P3 优先），轮速仍是 EKF 的主要输入。

### S — 快速验证（约 2 周，回答 80% 的问题）

1. **录数据**（第 1 周）：用 `scripts/record_3d_bag.sh`，已加入相机 IMU 话题，录 3 段，全部带雷达作真值：
   - 快速原地旋转（看跟踪是否稳定）；
   - 关灯或暗光（看暗光表现）；
   - 多房间大回环（看全局一致性）。
2. **4070S 离线对比**（第 2 周），同一份 bag 跑：
   - 基线：现有 RTAB-Map + EKF；
   - ORB-SLAM3 RGB-D-Inertial（S0 判定 IMU 可用时）或 RGB-D 模式；
   - MASt3R-SLAM（学习型稠密方法，只用彩色）。
3. **评测**：雷达 ICP 轨迹作参考，用 `evo` 算 ATE / RPE，统计跟踪丢失率。
4. 同时从 `/rtabmap/info` 记录 Pi 上的处理耗时，用来设 `TimeThr`。

**决策点**：
- VIO 明显优于基线，投 P2。
- 学习方法明显优于基线，投 P3。
- 都没有明显提升，说明瓶颈在相机硬件。按 A 档出结论，不再继续后面的阶段。

### P2 — VIO 前端（条件：S 显示 VIO 有收益）

| 方案 | 特点 |
|---|---|
| RTAB-Map `rgbd_odometry`（F2M）+ IMU 作初值 | 零新依赖，对照组 |
| ORB-SLAM3 RGB-D-Inertial | 成熟的紧耦合基线 |
| OpenVINS / Basalt | 最可能在 Pi 5 上实时跑 |
| VIW-Fusion 类（视觉 + IMU + 轮速） | 专治地面车尺度不可观；但我们的轮速是 cmd_vel 积分，收益需实测 |

- [ ] 选出的方案作为 EKF 的 `odom1` 接入：只融合 vx/vy/vyaw，跟丢时把协方差拉大。
- [ ] 测 Pi 5 能否实时跑；跑不动就放在 4070S 上，只服务建图，不服务控制。

### P3 — 后端上 4070S（条件：S 显示学习特征或学习方法有收益）

- [ ] Pi 和 4070S 用 chrony 对时，偏差 < 1ms。
- [ ] Pi 端发布 `rgbd_sync` 的压缩输出，降到 2–5Hz，带宽目标 < 2 MB/s。P0 已经引入 `rgbd_sync`，这一步只需要加压缩和节流。
- [ ] 4070S：rtabmap 带 torch 编译，用 SuperPoint（`Kp/DetectorStrategy=11`）+ SuperGlue（`Vis/CorNNType=6`），再加全局描述子回环。用 Docker 隔离。
- [ ] supervisor 加 `slam_3d_remote` 模式：后端在 4070S 上跑，`map→odom` 回传；断网时 Pi 端的 EKF 照常工作。
- [ ] 多光照、多会话建图（Labbé 2022 的做法）。

### P4 — 极端场景（条件：A/B 档已完成，用户明确要 C 档）

- [ ] 坡道：EKF 关 `two_d_mode` 并融合 roll/pitch，rtabmap 解除 `Force3DoF`。
- [ ] 暗光：改用 IR 流跟踪；投射器点阵会干扰特征，要么关投射器加外置红外补光，要么逐帧交替。外置补光要先评估 USB 供电余量。
- [ ] 隧道、长走廊：ICP 用点到面 + `Icp/PointToPlaneMinComplexity`；如实记录极限，不强求。

### P5 — 去雷达结论

- [ ] 全链路关闭 `subscribe_scan`，重跑所有已录场景，按场景给出"纯视觉够用 / 需要雷达 / 都不行"。

### 和自主探索的关系

本计划只负责地图的质量和鲁棒性，不包含自主探索。探索（Frontier + Nav2）另走 `feat/autonomous-navigation` 分支，它消费这里产出的 `/rtabmap/grid_map` 和 `map→odom`。P0 修好的栅格地图是探索的直接前提。

## 5. 风险与未知

| 风险 | 缓解 |
|---|---|
| Gemini 2 的 IMU 和图像不是硬件同步，或时间偏移不稳定 | P1 第一件事就测；不行就退到松耦合（VIO 不融 IMU，只靠 EKF） |
| 15fps 彩色对快速旋转不够 | 测 30fps 下 Pi 的 CPU 和 USB 余量；或者 VIO 改用 IR 流 |
| WiFi 带宽和延迟抖动 | 后端只需要 2–5Hz；断网时本地 EKF 兜底 |
| 无编码器，VIWO 收益有限 | 考虑后续加编码器或光流传感器（硬件项，单独评估） |
| rtabmap torch 编译复杂 | 用 Docker 隔离在 4070S 上 |
| 长直、重复纹理的隧道 | 纯视觉的物理极限，P5 如实记录，不强求 |

## 6. 下一步

1. 在 Pi 上部署 `feat/3d-mapping-p0` 分支，按 P0 的"需要在 Pi 上做"清单逐项验证。
2. 开 `camera_imu:=true`，跑 `scripts/check_camera_imu_sync.py`，做 S0 决策。
3. 录 S 阶段的 3 段 bag，带到 4070S 上做离线对比。

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
