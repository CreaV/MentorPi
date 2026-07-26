# Agent Handoff

- 更新时间：2026-07-26
- 分支：**`main`**（`feat/voice-vla-extensions` 已并入并删除，本地+远端）
- 工作树：clean，除未跟踪目录 `rtabmap_maps_pi/`（**用户自有，勿动**）
- 本会话内容：答疑「机器人描述文件在哪（Isaac 导入）」→ 发现文档里相机外参是过时值 → 修正 `CLAUDE.md` + `isaac/README.md` → 提交 → 并入 main + push → 删分支

本文是整个仓库唯一的滚动会话交接。下一次 agent 先读本文，结束时重写覆盖。

## 分支收敛（2026-07-26）

`feat/voice-vla-extensions` 的历史此前已由用户在 GitHub 通过 **PR #1 / #2** 合入 `origin/main`（合并提交 `cd4580b`、`3ad3775`，含到 `faa5b86`）。本会话本地还多两个提交，已 **rebase 到 `origin/main` 之上**（无冲突）并推送：

- `5ee34fc` Foxglove `loc_check` 布局增强（原 `960f147`）
- `691443e` 相机外参文档同步（本次改动）

`feat/voice-vla-extensions` 本地与远端分支均已删除。**配对仓库 AIRE 的 `feat/robot-skill` 是另一件事，见文末。**

## 本会话做了什么（2026-07-26）

**1. 答疑：Isaac Sim/Lab 的机器人描述文件**
产物在仓库根 `isaac/`：`mentorpi.isaac.urdf`（底盘+相机+雷达）、`mentorpi_so101.isaac.urdf`（整机+臂）、`mentorpi_articulation_cfg.py`（Isaac Lab `ArticulationCfg`）、`export_isaac.sh`、`README.md`。网格是**绝对路径**指向 `src/mentorpi_description/meshes/`，导入不需要 ROS 环境。导入设置：Fix Base Link **OFF**、Merge Fixed Joints **ON**。麦轮滚子 Isaac 不模拟。

**2. 文档相机外参修正（本次唯一代码/文档改动）**
`isaac/*.isaac.urdf` 实际已带 2026-07-24 AprilTag 标定值（07-24 21:50 重新导出过，与 `mecanum.xacro` 一致），但**文档还写着标定前的 vendor CAD 值 `0.061376 0 0.051154` 并标注「待标定」**。已改为实际值：

- `camera_joint` = `xyz 0.1017 0.0137 0.0535 / rpy -0.0171 -0.1196 0.0323`
- 修改点：`CLAUDE.md`（顶部 What This Is、TF 源头段、Static TF 表、Calibration 段、SO-101 段）、`isaac/README.md`（Frames 段）
- 验证：`grep -rn 0.061376 *.md` 现仅剩本文历史叙述外的 0 处代码/文档引用（xacro/URDF 本来就是新值）

**3. Git 操作**（详见「分支收敛」节）
提交 `691443e`（docs 相机外参同步 + 本交接）→ 本地先 ff 进 main → 发现 `origin/main` 已被 PR #1/#2 推进 → `git rebase origin/main`（无冲突）→ `git push origin main` → 本地 `git branch -D` + 远端 `git push origin --delete feat/voice-vla-extensions`。

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

## 自主探索调研（2026-07-26 前序会话，纯讨论 / 无代码 / 未落盘成独立文档）

用户提出「陌生环境自由探索 + 建图」需求，只做了技术调研。**用户已明确：目标产物 = 只关心 3D 地图；本轮只要调研不要实施计划。**

**关键结论（已联网核实）**：

1. **「真 3D 探索规划器」（TARE / FUEL / GBPlanner / DSVP）不适合本车硬件**。它们假设 360° **3D 激光**（TARE 官方基于 VLP-16）。本车只有 2D MS200 + 窄视场（~90°，有效 ~0.3–5m）Gemini 2，喂不动其「体积增益」逻辑。TARE 支持 ROS2 Jazzy + ARM，但**卡在传感器不是软件**。
2. **本车拿 3D 地图的务实路径 = 2D 决策探索 + RTAB-Map 3D 乘客**：探索决策在 2D（哪里是未知地面），产物含完整 3D 彩色地图（相机常驻，rtabmap 只从 TF 拿位姿）。Pi 5 友好。
3. **真正的前置工作是 Nav2 接麦轮底盘**（holonomic 控制器 + 代价地图调参），探索包本身很轻——所有 frontier 包都靠 Nav2 `navigate_to_pose` 执行目标点。
4. **候选探索包**：**`frontier_exploration_ros2`（mertgulerx）最匹配**——ROS2 Jazzy+Humble，CPU 3.5–8% / RAM 56MB，官方点名 Pi 友好，v1.6.1，测到 99.9% 覆盖；备选 `m-explore-ros2`（explore_lite，主 Humble 需验证 Jazzy）、`AniArka` Explorer+Mapper。

**复杂度分层**（回应用户「不就是扫地机逻辑吗」）：
- ① **反应式游走 + RTAB-Map**（≈早期扫地机）：几乎不用 Nav2，复用现有 `obstacle_guard`，最省，建议作「先跑起来」第一步。缺点：不保证扫全、可能打转。
- ② **全覆盖规划（弓字形）**（≈现代激光扫地机）：需 `opennav_coverage` / `full_coverage_path_planner`，中等工作量。
- ③ **Frontier + Nav2**：最智能，但 Nav2 麦轮集成是大头。

**地形边界条件（重要）**：
- **下行台阶 / 悬崖 / 桌边 = 危险失效**，与探索算法复杂度**正交**（Nav2 也不防）：2D 雷达在 ~14cm 高，对负障碍天生瞎；`obstacle_guard` + `/depth_scan` 只防**正障碍**，**对掉落零防护**；本车**无悬崖传感器**。→ 加防护前，自由探索只能在**确认无落差的单层平面**跑且有人盯。缓解：朝下红外悬崖传感器（最稳）/ 深度相机负障碍检测（相机朝前直装、近距 ~0.2m 盲区，需评估甚至改略俯视）/ 已知地图圈禁行区（对首次探索无用）。
- **多层楼梯**：车过不去 → 只有一层可达；无自动跨层拼接，需人工每层搬运单独建图。
- **陡坡**：车**过得去但地形是 3D 的** → **2D 从根上不成立**（雷达平面被掀斜→坡上误触发避障 / pose 丢 z 与 pitch / 2D 地图把坡与坡顶压叠）。缓坡可当噪声扛；真陡坡需 RTAB-Map 3D（已有）+ **2.5D 高程/可通行性分析**（`grid_map` / ETH `elevation_mapping` / Nav2 3D-aware costmap，项目里没有）。且**麦轮牵引/防翻是先卡住的物理天花板**（估计仅几度~十几度可靠，20°+ 很悬）。

**下一步候选（用户尚未选定）**：(a) 转向 ①/② 反应式游走+弓字形覆盖；(b) 先调研**掉落防护**（若现场有落差，这应最高优先）；(c) 调研 2.5D 高程/可通行性 + 麦轮爬坡；(d) 出 `frontier_exploration_ros2 + Nav2 麦轮` 可行性细节。

**来源**：github.com/caochao39/tare_planner、cmu-exploration.com、github.com/mertgulerx/frontier_exploration_ros2、AniArka Autonomous-Explorer-and-Mapper-ros2-nav2、abdulkadrtr ROS2-FrontierBaseExploration。

## Foxglove `loc_check` 布局增强（提交 `5ee34fc`，已在 main 并推送）

客户端 import 用的布局文件，改它不需要 colcon build，Foxglove 里 `File → Import layout` 重选即生效。共 17 个 panel，新增 6 类：

1. **深度图** `/viewer/depth_raw`（raw 16UC1，2Hz 节流）。⚠️ **压缩深度 Foxglove 渲染不了**（`/…/compressed` 发 0 字节；`/…/compressedDepth` 带 12B ConfigHeader 会黑屏，见 `remote.launch.py:117-123`）。彩色图才走压缩 `/viewer/color_compressed`。
2. **遥控** `Teleop!drive` → `/cmd_vel`（±0.25 m/s，±1.0 rad/s，10Hz），仍受 base_node 避障拦截。
3. **3D 建图命令** `setSlam3d`（新建）/`setSlam3dExtend`（续图 `load_all_nodes:true`）/`listMaps`。都是追加式 → **建新图须在 advanced 里改 `database_path` 为新文件名**。
4. **避障开关** `guardOn`/`guardOff` 调 `/mentorpi_base/set_parameters` 热更 `obstacle_guard`。节点名是 `mentorpi_base`（不是 base_node）。
5. **避障距离** `guardDist` 热更 `guard_stop_distance`/`guard_slow_distance`；另有原生 `Parameters!guard` 面板。
6. `Indicator!guard` 订 `/guard/blocked`；左侧 3D 场景补 `/rtabmap/cloud`。

`mentorpi.json` 主布局按用户意愿**不同步**这些面板。CallService 面板需真机 `remote.launch.py` 在跑。

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

## 下个 agent 如何继续

```bash
git status --short --branch
git log -5 --oneline --decorate
```

1. 现在在 **main** 上，与 `origin/main` 同步，工作树 clean（`rtabmap_maps_pi/` 未跟踪，用户自有）。所有分支已收敛，无待推送内容。
2. 读本文，再打开当前任务直接相关源码。
3. 配对仓库 AIRE（`/media/luo/Game/data/code/AIRE`）与本仓库的 `mentorpi_motion` 成对部署，状态见文末「AIRE 配对仓库」节。
4. 若继续自主探索：先向用户确认「下一步候选」选哪个（尤其掉落防护是否优先）。
5. 若继续 CAD/URDF：先改源（xacro / Python 生成器）、再生派生物、确定性检查+快照、URDF/无臂回归，最后更新本文。
