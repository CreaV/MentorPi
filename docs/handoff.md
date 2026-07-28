# Agent Handoff

- 更新时间：2026-07-28
- 分支：**`main`**（`feat/voice-vla-extensions` 已并入并删除，本地+远端）
- 工作树：本次交接提交后除未跟踪目录 `rtabmap_maps_pi/` 外 clean（该目录**用户自有，勿动**）
- 本会话内容：复核自主建图/VLA 导航方案，并评估 Qwen-RobotNav、OmniVLA、OmniNav 对 MentorPi + 4070S + RTAB-Map 的适配性。结论仍是**模型选探索目标、Pi 本地 Nav2/安全层执行的混合架构**；其中 OmniNav 的 Frontier slow-fast 设计最贴合语义探索。实机工作站的 4070S 能尝试运行其 3.88B Slow checkpoint，但显存余量不宽，且官方发布绑定 Habitat，并非 ROS2 真机框架。本轮仅调研答疑，无功能代码改动

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

## 自主探索调研（2026-07-27 复核；此前纯讨论 / 无功能代码）

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

**2026-07-27 代码状态复核**：`README.md` 只说明现有 `/map` 或 `/rtabmap/grid_map`、TF、`/scan`、`/cmd_vel` 已满足 Nav2 的输入/输出接口；`docs/roadmap.md` 仍把 Nav2 接入列为实施步骤。`src/` 中未找到 Nav2 配置、costmap/planner/controller、frontier、exploration 或 coverage 实现。因此当前能力仍是人工遥控建图/定位 + `obstacle_guard`，不是自主探索或自主导航。

### VLA/VLM + 3D SLAM 自由探索评估（2026-07-27，纯讨论）

- **可行，但不建议端到端 VLA 直接持续输出 `/cmd_vel`。** 推荐分层：Pi 5 常驻 EKF + RTAB-Map + Nav2 + costmap + watchdog/guard；局域网 4070S 只做低频视觉/语义判断、Frontier 候选排序、任务记忆和停止判定，向 Pi 下发 `NavigateToPose` goal 或有界 motion primitive。
- AIRE 已有可复用的 Stage-1：`air_engine/cloud/robot/vlm_agent.py` 每步取 `/viewer/color_compressed`，模型只选 `move/rotate/done`；`tools.py` 把动作限制成闭环 `MotionPrimitive`。但当前默认模型是远端 Gemini，输入只有当前 RGB + 文本动作历史，**没有地图、map 系位姿、Frontier、路径规划或覆盖率记忆**，只能做视觉游走/找目标，不能保证扫全。
- 推荐闭环：RTAB-Map 发布 `/rtabmap/grid_map` 并持续写 3D DB/点云 → 本地 exploration manager 提取可达 Frontier → 4070S 根据 `信息增益 + 路径代价 + 语义新颖度 + 风险` 选候选 → Nav2 本地到达 → 原地环视/主动观察 → 重复；无可达 Frontier、覆盖稳定、超时或低电量时结束。
- 不要把完整 `/cloud_map` 经 rosbridge JSON 高频喂给模型；传压缩 RGB、低频局部深度/栅格图和结构化 Frontier/pose 即可，3D 点云与 RTAB DB 留在机器人侧。断网必须由 Pi 本地取消 goal/停车。
- 4070S（标准 12GB）适合本地小型 VLM/SmolVLA 推理和受限微调；但 SmolVLA 是需要按自有机器人动作空间接入并用自有数据微调的 action-chunk 策略，不能拿基础 checkpoint 直接可靠驾驶 MentorPi。`pi0/pi0.5` 的微调显存需求明显超出 12GB，也不是当前底盘探索的优先路线。
- 实施顺序建议：① 先用现有 VLM + motion primitive 在封闭平地做受控 MVP；② 接入麦轮 Nav2；③ 加 Frontier/exploration manager 和地图状态接口；④ 再把 Gemini planner 换成本机 4070S 模型/比较模型。自由运行前仍需解决悬崖/下行台阶检测。

### 导航基础模型适配性复核（2026-07-27）

- **Qwen-RobotNav：架构高度匹配，但当前不可落地。** 官方模型基于 Qwen3-VL，输出 8 个 `(x,y,theta)` 局部 waypoint，支持单相机、PointNav/ObjectNav/VLN/跟踪和两层 agent + memory，接口很适合接 MentorPi 的 Nav2；但官方仓库明确暂不发布 RobotNav 权重，仓库也只有报告/演示，无推理代码，现阶段只能借鉴接口，不能部署。
- **OmniVLA：可运行原型，但它是 goal-conditioned 局部导航器，不是自由探索器。** 官方代码、8B BF16 checkpoint 和 50M `OmniVLA-edge` 均已发布；输入可用当前 RGB + 2D goal pose/目标图/语言，输出 8×4 waypoint chunk（机器人系 `x前/y左/cosθ/sinθ`），示例以 3Hz 推理并用 PD 转 `linear/angular`。RTAB-Map 的 map 位姿可替代 GPS；麦轮可先当差速底盘执行，若要横移必须改动作头并微调。完整 8B BF16 权重本身约 16GB，官方代码直接 BF16 上 GPU，12GB 4070S 不能原样加载；需非官方 4bit 量化/CPU offload，或先试 edge 版。训练 debug 官方要求至少 20GB，完整训练使用 8×H100。
- **OmniVLA 仍需外置 Frontier/coverage manager。** 它解决“给定目标怎么走”，不会自行以信息增益选择未知区域。且远端以 3Hz 直接发速度对 Wi-Fi/推理延迟敏感；若试验，应让它输出候选局部轨迹，由 Pi 本地碰撞检查/Nav2/安全盾执行，而不是直通 `/cmd_vel`。AsyncVLA 正是为此增加本地 Edge Adapter，但其训练代价更高；本项目可以先用确定性的 Nav2 充当 edge executor。
- **OmniNav：概念上最贴合目标。** 它以 Qwen2.5-VL-3B 做 slow-fast 系统，慢模块读取候选 Frontier、pose-stamped 图像记忆并选探索子目标，快模块生成连续 waypoint，论文明确包含 frontier-based exploration，并维护 3D occupancy map。4070S 规模合适，RTAB-Map 可提供地图/位姿；但公开代码主要绑定 Habitat R2R/RxR/OVON benchmark，移植真机 ROS2、替换其 3D occupancy/frontier 接口仍是明显工程量，不能当作即插即用 ROS 框架。
- 当前推荐排序：**近期工程落地 = 自建 Frontier + Nav2 + 本地 VLM 排序**；研究原型 = 移植 OmniNav slow system 或试 OmniVLA-edge；等待型 = Qwen-RobotNav 权重发布后再重新评估。无论选哪个，RTAB-Map 继续只负责地图/定位，模型不应绕过 Pi 本地 safety/goal lease。

### OmniNav 在本机的实测前评估（2026-07-27）

- 工作站实测：RTX 4070 SUPER **12282 MiB**（检查时空闲 10910 MiB）、内存 31 GiB、swap 8 GiB、数据盘空闲 474 GiB；PyTorch `2.7.1+cu126` 可识别 CUDA。硬件与 CUDA 基础满足推理。
- 已只在 `/tmp/omninav-inspect-20260727` 检出官方 OmniNav `10caac3` 做源码检查，并以跳过 LFS 的方式检查官方 Slowfast checkpoint 元数据；未下载大权重、未改项目源码。checkpoint 是 Qwen2.5-VL、BF16、约 3.88B，两个 safetensors 分片约 **5.0GB + 3.2GB**。官方 loader 用 `device_map="auto"`、BF16、`flash_attention_2`。据此判断：12GB 4070S **大概率能跑低频 Slow planner 单次推理**，但约 8.2GB 权重之外只剩约 4GB 给视觉 token/激活，需关闭其他 GPU 任务；OOM 时会 CPU offload 并明显变慢。此判断尚未用完整权重实跑验证。
- 当前默认 Python 环境**不能直接启动**：仅有 torch，未装 `transformers`、`qwen_vl_utils`、`flash_attn`、`habitat_sim`。官方 benchmark 还固定 Habitat-Sim v0.2.3 和定制 Habitat-Lab `v0.2.3_waypoint`；建议独立环境复现，避免污染 ROS/Python 3.12 环境。
- 官方 Slowfast 的 `fast_type=A-star` 实际是 **Qwen slow frontier selector + Habitat A*/GreedyGeodesicFollower**，很适合映射成 **Qwen slow selector + Pi 上 Nav2**。慢模块在到达 frontier 后原地 12×30° 扫描，再取 5 张方位图进行决策；单个固定前视 Gemini 2 可用底盘原地旋转替代多相机，但官方 Habitat 图像 HFOV 110°，Gemini 的较窄视场会形成域差异，需真机验证。
- 官方代码不能原样上真机：它从 Habitat ground-truth navmesh 生成 512×512 top-down map/fog-of-war/frontier，并直接使用 `habitat_sim.AgentState`/pathfinder；项目没有 ROS2 接口。真机适配必须用 RTAB-Map `/rtabmap/grid_map` 替代地图、自己提可达 frontier、记录 map-frame pose/image memory，再把选中的 subgoal 交给 Nav2。
- 更关键的任务差异：公开 Slow checkpoint 是 **OVON/ObjectNav**，prompt 是“寻找某个物体并在找到后停止”，不是“最大化覆盖率并完成整张 3D 地图”。所以 OmniNav 适合给 frontier 加**语义搜索排序**，不能单独承担完整覆盖和停止判定；无目标建图仍需确定性的 coverage/frontier manager。纯全屋建图时，它不是第一阶段必需项。
- 分层结论：① 官方 Habitat demo：硬件大概率够，但环境/数据安装重；② checkpoint 单次 slow decision：4070S 大概率可跑，值得做最小 smoke test；③ 直接控制 MentorPi：不行，需 ROS2 adapter；④ 完整训练：12GB 不现实。最值得做的是抽出 Slow Qwen 推理，不携带 Habitat，把 Nav2 当 fast executor。

### 导航分支策略（2026-07-28）

- 当前仍在 `main`；本轮只有调研交接文档，直接在 `main` 提交，不为了文档先建空分支。
- 真正开始 Nav2、Frontier、OmniNav ROS2 adapter 或安全状态机实现时，从最新 `origin/main` 新建 **`feat/autonomous-navigation`**。这会把尚未定型的导航栈与稳定的 3D SLAM 主线隔离开；无需复活已经合并删除的 `feat/voice-vla-extensions`。
- 建议该分支的第一阶段只做 **Nav2 麦轮 + 传统 Frontier 闭环**，通过真机安全验证后再接 4070S OmniNav Slow selector，避免同时调试底盘控制、探索逻辑和 VLM。

**来源**：github.com/caochao39/tare_planner、cmu-exploration.com、github.com/mertgulerx/frontier_exploration_ros2、AniArka Autonomous-Explorer-and-Mapper-ros2-nav2、abdulkadrtr ROS2-FrontierBaseExploration、github.com/amap-cvlab/OmniNav、modelscope.ai/models/chongchongjj/OmniNav_Slowfast。

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

## AIRE 配对仓库（`/media/luo/Game/data/code/AIRE`）

语音/VLA 的「大脑」在 AIRE，通过 rosbridge `:9090` 调本仓库 `mentorpi_motion` 的 `motion/primitive` action 和 `motion/stop` service —— **两仓库必须成对部署**（机器人端要有 `mentorpi_motion` 已构建且在 `base.launch.py` 里）。

**2026-07-26 本会话**：AIRE `feat/robot-skill` 已 `--ff-only` 合入 `main`（9 个提交，到 `c2516d0`）并 push 到 `origin/main`。**分支本身保留**（用户只要求删 MentorPi 那条）；local + `origin/feat/robot-skill` 都还在。AIRE 工作树有未跟踪的 `.venv/`、`package.json`、`package-lock.json`（**用户自有，勿动**）。

关键入口：`air_engine/cloud/skills/robot.py`（skill）、`air_engine/cloud/robot/{rosbridge_client,tools,vlm_agent}.py`、`robot_cli.py`、`robot_vla_cli.py`。服务器端设 `AIR_ROBOT_ROSBRIDGE_URL=ws://192.168.8.117:9090` 启用。手动验证：`python robot_cli.py ws://192.168.8.117:9090 move forward 0.5`。

## 下个 agent 如何继续

```bash
git status --short --branch
git log -5 --oneline --decorate
```

1. 现在在 **main** 上；本次调研交接已提交并推送。工作树除 `rtabmap_maps_pi/` 未跟踪目录外 clean，该目录用户自有。
2. 读本文，再打开当前任务直接相关源码。
3. 配对仓库 AIRE（`/media/luo/Game/data/code/AIRE`）与本仓库的 `mentorpi_motion` 成对部署，状态见文末「AIRE 配对仓库」节。
4. 若开始自主探索实现：先从最新 `origin/main` 新建 `feat/autonomous-navigation`；第一阶段优先做 Nav2 麦轮 + 传统 Frontier，并确认掉落防护边界，之后再接 OmniNav Slow selector。
5. 若继续 CAD/URDF：先改源（xacro / Python 生成器）、再生派生物、确定性检查+快照、URDF/无臂回归，最后更新本文。
