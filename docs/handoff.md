# Agent Handoff

- 更新时间：2026-07-25
- 分支：`feat/voice-vla-extensions`
- 关键提交（均已 push origin）：相机恢复直装（`84598e6`）、文档清理 Gemini 2L→2 + 云台（`728ce5c`）、Isaac 导出（`4410caf`）、**AprilTag 相机外参标定 RMS 9.6mm（`0846f7e`，已部署 Pi 并验证 TF）**。
- 工作树：**dirty** —— `src/mentorpi_supervisor/foxglove_layout/loc_check.json` 已改（本会话增强，未提交）；另有未跟踪目录 `rtabmap_maps_pi/`（用户自有，勿动）。
- 本会话内容：Foxglove `loc_check` 布局增强（见下节）+ 答疑（Gemini 2 深度/3D SLAM 角色）。

本文是整个仓库唯一的滚动会话交接。下一次 agent 先读本文，结束时重写覆盖。

## 本会话做了什么（2026-07-25）

**Foxglove `loc_check.json` 布局增强**（已改盘，`git status` 显示 `M`，**未提交**）。这是客户端 import 用的布局文件，改它不需要 colcon build，Foxglove 里 `File → Import layout` 重选即可生效。在原 loc 布局基础上加了 6 类面板（共 17 个 panel）：

1. **深度图**：`Image!depth` 订 `/viewer/depth_raw`（原始 16UC1，2Hz 节流，`lazy`）。⚠️ **压缩深度在 Foxglove 渲染不了**（`/…/compressed` 发 0 字节；`/…/compressedDepth` 带 12B ConfigHeader 会黑屏，见 `remote.launch.py:117-123`），所以只能用 raw 节流流。彩色图才走压缩 `/viewer/color_compressed`。
2. **遥控**：`Teleop!drive` 发 `/cmd_vel`（↑↓=±0.25 m/s，←→=±1.0 rad/s，10Hz）。裸速度流，仍受 base_node 避障兜底拦截。
3. **3D 建图命令**：`CallService!setSlam3d`（新建，`load_all_nodes:false`）、`setSlam3dExtend`（续图/扩图，`load_all_nodes:true`）、`listMaps`（`/mode/list_maps`）。新建/续图都是追加式 → 建新图须在 advanced 里把 `database_path` 改新文件名。
4. **避障开关**：`CallService!guardOn`/`guardOff` 调 `/mentorpi_base/set_parameters` 热更 `obstacle_guard`（bool，type=1）。节点名是 `mentorpi_base`（不是 base_node）。
5. **避障距离**：`CallService!guardDist` 热更 `guard_stop_distance`/`guard_slow_distance`（double，type=3，advanced 里改 `double_value`）；外加原生 `Parameters!guard` 面板可读/改 base_node 全部参数。
6. **附赠**：`Indicator!guard` 订 `/guard/blocked`（空=绿 CLEAR，非空=红 BLOCKED）；左侧 3D 场景补 `/rtabmap/cloud`（rgb 上色）看建图。

foxglove_bridge 已开 `parameters` capability（`remote.launch.py:97`），Parameters 面板和 set_parameters 按钮均可用。JSON 已 `python3 -c json.load` 校验通过。**主布局 `mentorpi.json` 未同步这些改动**（用户只要求改 loc_check）。

**答疑记录**：Gemini 2 深度**参与 3D 建图**（累积彩色点云 + 2D 栅格 + 回环几何校验的核心输入），但**不参与位姿**（`rgbd_odometry` 停用，pose 来自轮速+IMU EKF，~20ms/45Hz，白墙暗光不 fail）；相机自带 IMU 也不接入。异构设计：深度管「地图长啥样+回环校正」，轮速+IMU 管「此刻在哪」。

### 遗留（本节相关）
- `loc_check.json` 改动**未提交**，等用户决定是否 commit / 是否同步到 `mentorpi.json`。
- 布局里的 CallService 面板需真机 `remote.launch.py` 在跑才有对应 service；纯离线 Foxglove 只是按钮点了报错。

## 上一会话（2026-07-24）：相机外参标定 + 文档清理 + Isaac 导出

1. **交接机制泛化**（提交 `6e4aff4`，已推送）：`so101_handoff.md → docs/handoff.md`，`CLAUDE.md` 改为项目级，加 `AGENTS.md → CLAUDE.md` 软链。按用户明确指示 `git restore` 丢弃了 `loc_check.json` 的 Foxglove 布局改动。
2. **相机恢复直装外参**（提交 `84598e6`，**未推送**）：详见下节。
3. **相机外参重标（Part 3）完成并部署**（`0846f7e`）：AprilTag 手眼标定 RMS 9.6mm，写 `camera_joint` + visual 偏移，重生成 ROS/Isaac URDF，Pi pull+build+restart，实时 TF 已验证。见下节。
4. **文档清理**：`Gemini 2L → Gemini 2`（散文名 47 处；实机确认是 Gemini 2，非 2L）；2-DOF 云台移出文档（功能性 `/gimbal/cmd`·`Gimbal.msg` 保留，"云台舵机"改称通用 PWM 舵机）；CLAUDE.md 相机 TF 表旧值同步为 `0.061376`。删除 `TODO.md §4.7` 云台单目相机设想段。
5. **Isaac Sim/Lab 导出**：新增 `isaac/`，见「Isaac 导出」节。

## 已提交：相机恢复直装外参（`84598e6`，未推送）

2-DOF 云台已物理拆除，但 `camera_joint` 仍是云台时期的失效手眼值 → 相机在模型里悬空。从 `mecanum.pre_calibration.xacro` 恢复 vendor CAD 直装值：

- `camera_joint`：`xyz 0.1114 0.0305 0.0950 / rpy -0.0164 -0.1302 0.0147` → `xyz 0.061376 0 0.051154 / rpy 0 0 0`
- `camera_link` visual+collision origin：`-0.0137 -0.0305 0.013` → `0 0 0`（成对归零：camera_link 回归网格本体原点）
- 改**源** `mecanum.xacro`，再经 `cad:urdf` launcher 重生成 `mentorpi_so101.urdf` / `.viewer.urdf`；`preview.glb` 重烤（gitignore，未入库）
- 验证：`check_urdf` ×2、`colcon build mentorpi_description`、runtime+arm-off 回归、旧值 grep=0 全部通过
- **性质**：粗定位。真实外参仍需下面的 AprilTag 重标（用户实测相机安装「可能有位移」）

坑（已修）：XML 注释里不能含 `--`，最初把 `--update-xacro` 写进注释导致 xacro 解析失败。

## 相机外参重标（Part 3）—— ✅ 完成并部署（2026-07-24）

**结果**：`camera_joint xyz=0.1017 0.0137 0.0535 / rpy=-0.0171 -0.1196 0.0323`（光学中心；z=0.0535 尺量固定），position-only RMS **9.6mm**，7 帧（3 弧站 ±11° + 4 直线站，窄空间用新增 `--far-dist 1.5`；±22° 在 0.9m 近距离 tag 转出视野被跳过）。`camera_link` visual/collision 偏移改 `-0.0407 -0.0124 0.0023` 把网格本体拉回 CAD 位（纯外观，与标定分离）。已写盘、重生成全部 ROS+Isaac URDF、提交 `0846f7e` 推送、Pi 部署（pull+build+`systemctl restart mentorpi-remote`），实时 TF 验证 `[0.102,0.014,0.053]` 一致。**注意**：camera_info 经 rosbridge QoS 不稳,用 `--intrinsics 518.6 518.6 317.2 236.2` 绕过。

以下为执行记录（参数、坑，供追溯）：

方法：`scripts/calibrate_camera_extrinsic.py`，dev 机经 rosbridge 驱动小车自动 AprilTag 手眼标定（arc 站定原地转 + line 直线基线求 yaw）。Part 1+2 里程计/IMU 已签核（base_link 运动可信，前提满足）。

### 已确定的标定参数（用户提供/仓库记录）

- **标定板**：`tag36h11`，**ID = 0**，平板显示、稳靠墙面、良好光照。
- **`tag-size = 0.1175` m**（黑方块边长 117.5 mm）。这是**已有记录值**，非本次现测——脚本默认、`docs/calibration_handoff.md`、记忆 `calibration-state.md` 里历次成功标定（07-12/07-17，残差 ~9–10 mm）都用它。
- **`cam-z = 0.0535` m**（相机光心高出 base_link）。推导：用户尺量光心**离地约 10.4 cm**；`base_footprint→base_link` z = **0.0505 m**（离地）；故 `0.104 − 0.0505 = 0.0535`。与直装 CAD 标称 0.051 只差 2.5 mm，互证。⚠️ **脚本默认 `--cam-z 0.095` 是旧云台高度，禁用**。z 在解里固定不可观测，全靠此手填值。

### 运行命令（电池充好、用户确认安全后）

```bash
# venv 已就绪：/media/luo/Game/data/code/AIRE/.venv
/media/luo/Game/data/code/AIRE/.venv/bin/python \
  scripts/calibrate_camera_extrinsic.py ws://192.168.8.117:9090 \
  --tag-size 0.1175 --cam-z 0.0535 --out /tmp/calib_run
# 先看残差 RMS，质量 OK（<20mm）再单独 --solve-only /tmp/calib_run --update-xacro --max-update-rms-mm 20
# 写回后：colcon build mentorpi_description → 重生成 URDF（同 84598e6 流程）→ Pi 部署
```

**第一次不要 `--update-xacro`**：先出残差/外参给用户看，确认后再写盘。

### ⚠️ 空间受限 —— 默认采集流程跑不动，必须改

用户现场：车**后方仅 ~1 m**、**前方 ~0.8 m**（板在前墙），且**远了看不清 tag**。脚本 `collect()` 默认要退到 **1.6 m 基线**，这里达不到。

需要把 `collect()` 里两个硬编码距离阈值调小以适配（当前无 CLI 开关）：
- 后退目标 `1.6`（`while d ... < 1.6`，约 296–302 行）→ 降到 ~`1.3`（或更保守，保证退到时 tag 仍清晰可检）
- 直线站近端下限 `1.0`（`if d < 1.0: break`，约 310 行）→ 降到 ~`0.8`，让前进段能从远端采到起始位；**绝不可比起始位（~0.8 m）更靠近平板**
- 避障守卫**全程保持开启**（历史上关守卫 + 固定步数前进曾撞倒平板，勿回退）

短基线会削弱 yaw 观测精度 → 残差可能高于历史 9–10 mm；以实际 RMS 决定是否写盘。建议先跑一个**零运动探测**（连上抓帧、检测 tag、报起始距离与 decision margin，不驱动车）确认起始可检测范围，再规划缩短版基线。

### 开跑前仍需用户现场确认

1. 电池充好、小车启动、正对标定板
2. **相机已刚性固定**（标定只在相机之后不动时有效）
3. 场地：车后方留出退让、板稳靠墙
4. 小车在 **idle** 模式（base+motion+rosbridge 在跑、无 SLAM 抢 map→odom）

### 环境就绪状态（本会话已备好）

- venv `/media/luo/Game/data/code/AIRE/.venv`：`pupil_apriltags / scipy / Pillow / numpy / websockets` 全部导入通过
- AIRE 仓库 `/media/luo/Game/data/code/AIRE`（脚本 `--aire-path` 默认指向它，靠 `sys.path` 导入 `air_engine.cloud.robot`）
- 小车 `192.168.8.117` ping 通、rosbridge `:9090` 开着

## URDF/整机预览的局域网访问方式

用 `cad:cad-viewer` 技能的 viewer 后端本地渲染 URDF/GLB。默认 `--host 127.0.0.1` **只能本机访问**；要让局域网别的机器看，改绑 `0.0.0.0`：

```bash
# 从 viewer 目录启动（技能缓存路径）：
cd /home/luo/.claude/plugins/cache/text-to-cad/cad/0.3.9/skills/cad-viewer/scripts/viewer
node backend/server.mjs --host 0.0.0.0 --port 4178 \
  --dir /media/luo/Game/data/code/MentorPi/src/mentorpi_description/urdf
```

- dev 机局域网 IP：**`192.168.8.137`**（与小车同 `192.168.8.x` 网段）
- 局域网 URL（任意同网段客户端浏览器打开）：
  - URDF：`http://192.168.8.137:4178/?dir=/media/luo/Game/data/code/MentorPi/src/mentorpi_description/urdf&file=mentorpi_so101.urdf`
  - 整机可靠视图（baked GLB，绕过打散 fixed-joint 的 URDF loader）：同 URL 换 `file=mentorpi_so101.preview.glb`
- `?dir=` / `file=` 是**服务端（dev 机）路径**，故从哪台客户端打开都用这串绝对路径；`file=` 相对 `--dir`
- `preview.glb` 被 `.gitignore`（`*.preview.glb`），仅本地预览；需要时用 `/tmp/mentorpi-cad-venv/bin/python mechanical/urdf/bake_urdf_glb.py <urdf> <glb>` 重烤（venv 需 trimesh+scipy）
- 安全：裸跑无认证，假设局域网可信

## Isaac Sim / Isaac Lab 导出

新增 `isaac/`，可直接导入 Isaac Sim/Lab，**与标定/URDF 一键绑定**：

- 产物：`isaac/mentorpi.isaac.urdf`（底盘+相机+雷达）、`isaac/mentorpi_so101.isaac.urdf`（整机+臂）、`isaac/mentorpi_articulation_cfg.py`（Isaac Lab `ArticulationCfg`）。
- **绑定核心**：`bash isaac/export_isaac.sh` 从 `mecanum.xacro` 一键重生成两者（build+source+生成+`check_urdf`）。**相机重标 `--update-xacro` 后必须跑一次**再重转 USD。已验证两 URDF 带标定值 `0.061376`。
- 生成器 `mechanical/urdf/gen_mentorpi{,_so101}_isaac.py`；网格用绝对路径（导入无需 ROS）；`isaac/usd/` 已 gitignore。
- 导入步骤（浮动底座 + merge fixed joints）、麦轮滚子 Isaac 不模拟等注意事项见 `isaac/README.md`。README.md 和 CLAUDE.md「Extending the System」均有索引。

## 仍遗留（实物/部署，多数需硬件）

1. **3D 地图重扫**（相机重标的后续）：新外参 z 从 0.095→0.0535（降 4.2cm，云台抬高没了），旧 `room_20260717.db` 相机偏高约 4cm → 点云整体偏高。切 `slam_3d` 新建 db 重扫（多角度多回环），GS 数据集导出同理需重来。
2. **甲板干装**：四孔下方能否放垫圈/螺母、按底盘板厚+8mm 选 M4 螺丝长度、查线束干涉。
3. **2S 电池托架**：实测中层甲板厚度和电池尺寸，更新 `battery_tray_2s.py` 的 `HOOK_THROAT`/`PACK_*`。
4. **Pi 部署（SO-101）**：装 `ros-jazzy-laser-filters`，`with_so101:=true` 启动，验证 `/scan_raw → scan_mask → /scan` 的 ±24° 自体掩膜。
5. **地图**：相机重标 + 装臂 + 扫描高度变化后重扫 2D/3D；旧 3D 数据视为失效。
6. **后续**：2S 电气台架、Feetech 舵机 ID/零位/限位、LeRobot 接入；需规划时再做 MoveIt SRDF。
7. **相机型号功能引用**（见 `TODO.md §4.7`）：实机是 Gemini 2（非 2L），但 `camera.launch.py:26` 的 `gemini2L.launch.py`、`README.md` 的 `camera_type:=gemini2l`、USB PID `2bc5:0670` 仍是 2L。**当前运转正常**，改动需在 Pi 上核对（orbbec 是否有 `gemini2.launch.py`、`lsusb` 的真实 PID）后再动。

## 关键文件与命令

- 会话交接：`docs/handoff.md`（本文）
- Agent 规则：`CLAUDE.md`；`AGENTS.md` 为其软链
- 相机标定脚本：`scripts/calibrate_camera_extrinsic.py`；文档 `docs/calibration.md` Part 3 / `docs/calibration_handoff.md`
- SO-101 装配源：`src/mentorpi_description/urdf/mecanum.xacro`（相机/雷达/IMU 静态 TF 的源头）
- URDF 再生成：`cad:urdf` launcher，源 `mechanical/urdf/gen_mentorpi_so101{,_viewer}.py`；生成前需 `colcon build mentorpi_description` 让 `$(find)` 命中新装 xacro，并显式 `export AMENT_PREFIX_PATH=<repo>/install/mentorpi_description:/opt/ros/jazzy`
- Isaac 导出：`isaac/README.md`（一键 `bash isaac/export_isaac.sh`）；源 `mechanical/urdf/gen_mentorpi{,_so101}_isaac.py`；Lab 配置 `isaac/mentorpi_articulation_cfg.py`
- 甲板 CAD / 间隙 / 掩膜：`mechanical/printable/so101_deck_plate.py`、`mechanical/measurements/check_so101_clearances.py`、`compute_scan_mask_so101.py`

## 下个 agent 如何继续

```bash
git status --short --branch
git log -5 --oneline --decorate
```

1. 先分用户/agent 改动；本会话工作树应为 clean。
2. 读本文，再打开当前任务直接相关源码。
3. 若继续相机标定：确认电池+安全前置 → 改 `collect()` 两个距离阈值适配窄空间 → 先零运动探测 → 采集 → 看 RMS → 用户确认后 `--update-xacro` + 重建 + 重生成 URDF + Pi 部署。
4. 若继续 CAD/URDF：先改源、再生派生物、确定性检查+快照、URDF/无臂回归、最后更新本文。
