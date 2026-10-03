# Agent Handoff

- 更新时间：2026-10-03
- 分支：`feat/3d-mapping-p0`（基于 `docs/vision-mapping-roadmap`，后者基于 `main` `1380831`）；两个分支都已推送，都没开 PR、没合入 main
- 运行环境：本会话在**云端容器**的新克隆上进行，不是 dev 机，也没有 Pi 硬件。用户自有的未跟踪目录 `rtabmap_maps_pi/` 在 dev 机上，这里没有；回到 dev 机仍然**勿动**。
- 工作树：clean（本会话所有改动已提交并推送）

本文是整个仓库唯一的滚动会话交接。下一次 agent 先读本文，结束时重写覆盖。

## 本会话（2026-10-03）：3D 建图审查 + 纯视觉建图极限 roadmap + P0/S0 代码

### 0. 代码改动（`feat/3d-mapping-p0`，**未在 Pi 上构建或运行**）

这个容器里没有 ROS，只做了 py_compile、flake8 和纯 Python 的 pytest（`scripts/tests/test_check_camera_imu_sync.py` 8 个、`src/mentorpi_bringup/test/test_rtabmap_params.py` 7 个，全部通过）。**colcon build 和 launch 都没跑过。**

- `mentorpi_bringup/rtabmap_params.py`：slam_3d / loc_3d 共用的 rtabmap 参数，取代原来两份手工同步的副本。
- 新增 `rgbd_sync` 节点；rtabmap 改为 `subscribe_rgbd` + scan，话题 `/camera/rgbd_image`，`approx_sync_max_interval=0.034`。
- rtabmap Node 设 `sigterm_timeout=85` / `sigkill_timeout=15`，修复数据库落盘时被 SIGKILL 的问题。
- `Grid/MaxGroundHeight` 从 0.05 改为 -0.0105（相对 `base_link`，即离地约 4cm）。
- `camera_imu:=true` 参数链：remote → base → camera_watchdog 的 `launch_args` → `camera.launch.py` 的 `enable_imu`（附带 `imu_rate`，默认 200hz）。
- `record_3d_bag.sh` 增加相机 IMU 话题、`/scan_raw`、`/rtabmap/info`。
- 新增 `scripts/check_camera_imu_sync.py`：S0 判定，看时钟域、IMU 抖动、彩色与深度的偏移。
- `CLAUDE.md` 同步了上述变化；`docs/vision_mapping_roadmap.md` 改为修订版，加入目标档位 A/B/C 和"先快速验证、再按结果分支"的顺序。

**Pi 上的验证清单**见 roadmap P0 的"需要在 Pi 上做"和 S0。重点检查：
1. `colcon build --packages-select mentorpi_bringup mentorpi_supervisor` 通过。
2. `slam_3d` 能起来，`/camera/rgbd_image` 有数据，rtabmap 在处理。
3. 切到 idle 时日志里没有 SIGKILL。
4. 栅格地图的地板没有出现假障碍。
5. `camera_imu:=true` 时有 `/camera/gyro_accel/sample`；话题名是按 Orbbec v1 驱动推断的，需要实测确认。

以下 1–3 节是本会话的讨论结论，都来自读源码。

### 1. 3D 建图代码审查发现（第 1、2、4、6、7 项已在 `feat/3d-mapping-p0` 上改；第 3、5 项要等 Pi 数据）

1. **数据库可能仍会在落盘时被强杀。**supervisor 给 3D 模式 SIGINT 后等 90s（`supervisor_node.py:227`），但中间那层 `ros2 launch` 默认约 5s 转 SIGTERM、再过约 5s 发 SIGKILL，仓库里没有设 `sigterm_timeout` / `sigkill_timeout`。clock-jump-guard 的重启也依赖这个宽限。实际修法：在 rtabmap 的 `Node(...)` 上直接传 `sigterm_timeout` / `sigkill_timeout`，不依赖 launch 参数的作用域。
2. **地面阈值偏高 5cm。**`Grid/MaxGroundHeight: 0.05` 是相对 `base_link` 的，而 `base_link` 离地 0.0505m，所以实际阈值约离地 10cm，低障碍可能被当成地面。
3. **相机俯仰角和低障虚拟激光不一致。**camera_joint 的 pitch = -0.1196 rad，即抬头约 6.9°；`camera.launch.py` 的 depth_low_scan 却按"无俯仰"设计（注释里的高度 0.18 / 0.095 也已过时，实际是 0.143 / 约 0.104）。需要先用平地点云核实旋转标定，再考虑改成 `base_link` 系的高度切片。
4. RGB-D 同步太宽松：没设 `approx_sync_max_interval`，驱动没开帧同步，没用 `rgbd_sync`。
5. 没设 `Rtabmap/TimeThr` / `MemoryThr`，长时间建图无上限。
6. `Reg/Strategy: '1'` 的注释写"Visual + ICP"，实际值是纯 ICP（视觉回环先由视觉给初值）。
7. 可选：关 `Grid/3D` 省 CPU；`point_cloud_xyzrgb` 降频；`slam_3d` / `loc_3d` 的参数抽成共享 YAML；`rtabmap_mapping.launch.py` 没透传 `load_all_nodes`。

### 2. 坡道和隧道适配性（讨论结论）

当前架构只适合**单层平地**：EKF `two_d_mode`、rtabmap `Force3DoF`、2D 雷达三处都假设车是水平的。

- **坡道**：雷达倾斜后会看到假墙，打到地面的距离约为 0.143/tan(坡度)。
- **长直隧道**：激光 ICP 沿通道方向退化。
- **窄通道**：obstacle_guard 前方扇区 ±45° 按径向距离判断。车在正中时，通道宽度小于约 0.85m 就减速，小于约 0.42m 前进被清零。

### 3. 纯视觉建图极限：调研与 roadmap

- 用户的两条 B 站视频（OAK 甩流星锤、迪士尼过山车）在本容器里打不开，b23.tv、bilibili、CSDN、Luxonis 论坛都被网络策略拦截，只能用搜索结果。
- 过山车方案是因子空间 Factor Perception SDK：HF-Net 在相机端 NPU 上跑 + 紧耦合 VIO，**后端是 RTAB-Map**。Luxonis 官方示例也是 Basalt VIO + RTAB-Map。
- **结论：我们和它的差距在前端（VIO），不在后端。**
- 用户明确：**不换 OAK**；目标是探索不用激光雷达时建图能做到什么程度；可以用局域网里的 4070S 分担算力。
- 产出 `docs/vision_mapping_roadmap.md`，分 P0–P5：
  - P0：修上面的审查问题。
  - P1：评测基础设施。打开 Gemini 2 的 IMU / IR，用 Kalibr 标相机-IMU，Pi 和 4070S 用 chrony 对时，录 8 类刁难场景，**雷达只作真值**，用 evo 打分。
  - P2：VIO 前端对比，选中的方案作为 EKF 的 odom1 接入。
  - P3：rtabmap 后端放到 4070S，上 SuperPoint/SuperGlue 和全局描述子，解除 3DoF。
  - P4：暗光（IR）和极端场景。
  - P5：去掉雷达后的对比结论。

### 下一步

1. 在 Pi 上部署 `feat/3d-mapping-p0` 并按上面的清单验证。验证通过后再合入 main：可以开 PR，也可以直接合并，由用户决定。
2. S0：开 `camera_imu:=true`，运行 `/usr/bin/python3.12 scripts/check_camera_imu_sync.py`（先静止跑一次，再原地旋转跑一次），按 roadmap 的决策表选择方向。
3. S：录 3 段 bag，带到 4070S 上做离线对比。
4. 还没做：`Rtabmap/TimeThr`（等 `/rtabmap/info` 的耗时数据）、depth_low_scan 的俯仰问题（等平地点云核实）。

## 当前标定状态（权威值）

| 项 | 值 | 来源 |
|---|---|---|
| wheelbase / track_width（有效值） | 0.1528 / 0.1575 | 2026-07-16 对墙旋转标定 |
| wheel_diameter | 0.0636 | 2026-07-05 卷尺 |
| gyro_scale_z | 0.9930 | 2026-07-16 |
| `camera_joint` | `xyz 0.1017 0.0137 0.0535 / rpy -0.0171 -0.1196 0.0323` | 2026-07-24 AprilTag 手眼，position-only RMS **9.6mm**，7 poses；z 为尺量固定（平面运动不可观），**已部署 Pi 并验证实时 TF** |
| `laser_joint` | `xyz -0.012242 0 0.092501` | 直装恢复值，扫描面离地 143.001mm |
| `imu_joint` | `xyz 0 0 0.05` | **估计值**，Part 2 标定未做 |

源头唯一：`src/mentorpi_description/urdf/mecanum.xacro`。任何标定改动后必须 `bash isaac/export_isaac.sh` 重生成 Isaac URDF，再重转 USD。

## 自主探索 / VLA 导航调研（2026-07-27～28，仍有效，摘要）

- 推荐**混合架构**：模型（4070S）只选探索目标，Pi 本地 Nav2 和安全层执行。不要让 VLA 直接持续输出 `/cmd_vel`。
- 近期落地路线是自建 Frontier + Nav2 麦轮 + 本地 VLM 排序。OmniNav 的 slow selector 概念最贴合，但它绑定 Habitat，需要写 ROS2 适配器。OmniVLA 8B 用 BF16 放不进 12GB 显存。Qwen-RobotNav 还没发布权重。
- 开始实现时从最新 `origin/main` 新建 `feat/autonomous-navigation`。
- 前置条件：掉落防护（2D 雷达和低障守卫都对负障碍完全看不见）。
- 详细评估见 git 历史中本文的 `1380831` 版本（`git show 1380831:docs/handoff.md`）。

## 仍遗留（实物/部署，多数需硬件）

1. **3D 地图重扫**（相机重标的后续）：新外参 z 从 0.095→0.0535（降 4.2cm），旧 `room_20260717.db` 相机偏高约 4cm → 点云整体偏高。切 `slam_3d` 新建 db 重扫，GS 数据集导出同理需重来。
2. **IMU 标定 Part 2**：`imu_joint xyz 0 0 0.05` 仍是估计值，未做安装角精化。
3. **甲板干装**：四孔下方能否放垫圈/螺母、按底盘板厚+8mm 选 M4 螺丝长度、查线束干涉。
4. **2S 电池托架**：实测中层甲板厚度和电池尺寸，更新 `battery_tray_2s.py` 的 `HOOK_THROAT`/`PACK_*`。
5. **Pi 部署（SO-101）**：装 `ros-jazzy-laser-filters`，`with_so101:=true` 启动，验证 `/scan_raw → scan_mask → /scan` 的 ±24° 自体掩膜。
6. **后续**：2S 电气台架、Feetech 舵机 ID/零位/限位、LeRobot 接入；需规划时再做 MoveIt SRDF。
7. **相机型号功能引用**（`TODO.md §4.7`）：实机是 Gemini 2（非 2L），但 `camera.launch.py:26` 的 `gemini2L.launch.py`、`README.md` 的 `camera_type:=gemini2l`、USB PID `2bc5:0670` 仍是 2L。**当前运转正常**，改动需在 Pi 上核对后再动。

## 关键文件与命令

- 会话交接：`docs/handoff.md`（本文）；Agent 规则：`CLAUDE.md`（`AGENTS.md` 为其软链）
- 静态 TF / 几何唯一源头：`src/mentorpi_description/urdf/mecanum.xacro`
- Isaac 导出：`isaac/README.md`，一键 `bash isaac/export_isaac.sh`；生成器 `mechanical/urdf/gen_mentorpi{,_so101}_isaac.py`
- ROS URDF 再生成：`cad:urdf` launcher，源 `mechanical/urdf/gen_mentorpi_so101{,_viewer}.py`；生成前 `colcon build --packages-select mentorpi_description`，并 `export AMENT_PREFIX_PATH=<repo>/install/mentorpi_description:/opt/ros/jazzy`
- 相机标定：`scripts/calibrate_camera_extrinsic.py`；文档 `docs/calibration.md` Part 3。标定 venv `/media/luo/Game/data/code/AIRE/.venv`；⚠️ 脚本默认 `--cam-z 0.095` 是旧云台高度，**必须传 `--cam-z 0.0535`**；`--tag-size 0.1175`；camera_info 经 rosbridge QoS 不稳，用 `--intrinsics 518.6 518.6 317.2 236.2` 绕过
- 甲板 CAD / 间隙 / 掩膜：`mechanical/printable/so101_deck_plate.py`、`mechanical/measurements/check_so101_clearances.py`、`compute_scan_mask_so101.py`
- 整机可视化：`mechanical/urdf/bake_urdf_glb.py` 烤 GLB（three.js URDF loader 会打散 fixed-joint 子树）；viewer 绑 `0.0.0.0` 可局域网看，dev 机 IP `192.168.8.137`
- Pi：`pi@192.168.8.117`，部署 = pull + `colcon build` + `sudo systemctl restart mentorpi-remote`

## AIRE 配对仓库（`/media/luo/Game/data/code/AIRE`）

语音/VLA 的「大脑」在 AIRE，通过 rosbridge `:9090` 调本仓库 `mentorpi_motion` 的 `motion/primitive` action 和 `motion/stop` service —— **两仓库必须成对部署**（机器人端要有 `mentorpi_motion` 已构建且在 `base.launch.py` 里）。

**2026-07-26 本会话**：AIRE `feat/robot-skill` 已 `--ff-only` 合入 `main`（9 个提交，到 `c2516d0`）并 push 到 `origin/main`。**分支本身保留**（用户只要求删 MentorPi 那条）；local + `origin/feat/robot-skill` 都还在。AIRE 工作树有未跟踪的 `.venv/`、`package.json`、`package-lock.json`（**用户自有，勿动**）。

关键入口：`air_engine/cloud/skills/robot.py`（skill）、`air_engine/cloud/robot/{rosbridge_client,tools,vlm_agent}.py`、`robot_cli.py`、`robot_vla_cli.py`。服务器端设 `AIR_ROBOT_ROSBRIDGE_URL=ws://192.168.8.117:9090` 启用。手动验证：`python robot_cli.py ws://192.168.8.117:9090 move forward 0.5`。

## 下个 agent 如何继续

```bash
git status --short --branch
git log -5 --oneline --decorate
```

1. 读本文；当前主线是 3D 建图优化 + 纯视觉 roadmap，入口是 `docs/vision_mapping_roadmap.md`。
2. 改 rtabmap 参数时同步改 `slam_3d.launch.py` 和 `loc_3d.launch.py`，或者先把它们抽成共享 YAML。
3. 涉及 Pi 的验证：`pi@192.168.8.117`，部署 = pull + `colcon build` + `sudo systemctl restart mentorpi-remote`。
