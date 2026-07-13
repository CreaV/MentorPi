# 扩展路线图：语音控制 / VLA / 机械臂

2026-07-03 规划。三个扩展方向的可行性评估、目标架构与分阶段实现步骤。
原则：**Pi 只做实时安全层，所有智能放内网 GPU 服务器**，通信复用现有
rosbridge (:9090)。

## 可行性结论（TL;DR）

| 方向 | 可行性 | 关键决策 |
|------|--------|----------|
| 1. 语音控制 (AIRE) | ✅ 高，工作量最小 | 机器人 = AIRE 的一个 Skill 模块；Pi 上只跑"薄"边缘端（唤醒+VAD），ASR/Agent 全在服务器 |
| 2. VLA + 语音融合 | ✅ 可行，但**不选 Alpamayo** | 底盘移动用 "VLM 规划 + Nav2 执行"（分层），端到端 VLA 留给机械臂 |
| 3. 机械臂 (lerobot) | ✅ 可行 | **不走 STM32**——SO-101 的 Feetech 总线舵机用 USB 适配器直连 Pi，保住 lerobot 全生态 |

前置依赖：`TODO.md` 里的标定 Part 1-3（VLA/导航的执行精度底座）+ Nav2 接入
（CLAUDE.md 已列出全部输入就绪：`/map`、odom TF、`/scan`、`/cmd_vel`）。

## 目标架构

```
┌─ 机器人 (Pi 5) ── 实时层，无智能 ──────────────────────────────┐
│ base_node(串口安全: cmd_vel watchdog/deadman)  EKF  slam/loc    │
│ Nav2(局部避障+路径跟踪)  motion_primitive(有界动作执行器)        │
│ lerobot FeetechMotorsBus(USB→SO-101, 阶段3)                     │
│ AIRE edge thin(USB麦克风: openwakeword+VAD, 音频流上推)  TTS播放 │
└──────────────── rosbridge :9090 / 音频 ws ─────────────────────┘
                              ↕ 内网 WiFi
┌─ 内网 GPU 服务器 ── 全部智能 ──────────────────────────────────┐
│ AIRE cloud: faster-whisper ASR → Agent Loop → SkillRegistry     │
│   ├─ robot_skill: set_mode/goto/move/stop/arm_task (roslibpy)   │
│   └─ home_assistant 等既有 skill                                │
│ VLM (Qwen2.5-VL @ vLLM): 场景理解/目标grounding/任务规划        │
│ VLA/策略推理 (阶段3: ACT/SmolVLA/pi0, action chunk 下发)        │
│ 3DGS 训练 (已有)                                                 │
└─────────────────────────────────────────────────────────────────┘
         ↕
┌─ 客户端 ─ 手机SPA / Foxglove / rerun (已有) ─┐
```

安全模型（跨网控制的铁律）：服务器下发的永远是**有界意图**（"前进 0.5m"
"去点位 A""执行抓取任务"），不是裸 `cmd_vel` 长流；Pi 本地的 watchdog
（0.5s cmd_vel 超时停车，已有）+ Nav2 局部避障 + 新增 motion_primitive
的里程计闭环，保证断网 = 停车。

---

## 方向 1：语音控制（接入 AsynchronousIntentRoutingEngine）

AIRE 现状（`/media/luo/Game/data/code/AsynchronousIntentRoutingEngine`）：
端侧(唤醒/VAD/快速ASR) + 云端(faster-whisper/LLM Agent/SkillRegistry) +
端侧 TTS，skill 以 `ToolSpec + ToolHandler` 注册（见
`air_engine/cloud/skills/registry.py`，`home_assistant.py` 是现成范例）。

**集成方式：机器人 = 一个新 skill 模块**，服务器上用 roslibpy 连机器人
rosbridge，机器人侧**零改动**（rosbridge 已常驻）。

### 实施步骤

1. **motion_primitive 节点**（机器人侧，新 ROS 包）：吃
   `MotionPrimitive.action`（前进/横移 x 米、旋转 x 度、限速限时），用
   `/odometry/filtered` 闭环，完成或超时自动停。这是语音/VLA 共用的
   执行底座——语音说"往前走一点"不能靠定时发 cmd_vel。
2. **AIRE robot skill**（服务器侧，`air_engine/cloud/skills/robot.py`）：
   - `robot.move(direction, distance)` → motion_primitive action
   - `robot.rotate(angle)` / `robot.stop()`（stop 直通 `/cmd_vel` 零速）
   - `robot.set_mode(mode, map)` → `/mode/set`
   - `robot.status()` → `/mode/status` + 电量(后续) + 位姿
   - `robot.goto(place)` → Nav2（阶段 2 解锁）
3. **麦克风**：机器人加 USB 麦克风（ReSpeaker 2-Mic 或普通 USB 会议麦）,
   Pi 跑 AIRE edge 薄模式：openwakeword(唤醒) + webrtcvad(端点) + 音频段
   直推服务器 ASR（内网网络好，跳过端侧 sherpa ASR，省 Pi CPU）。TTS 回放
   加 USB 小喇叭。Pi CPU 预算：唤醒+VAD ≈ 5-10%，可接受（slam_3d 时总
   占用 ~75%/400%）。
4. **快慢双路**（AIRE 原生设计）："停""往前走"走 FastLoop 低延迟直达；
   "去厨房看看桌上有什么"走 Agent Loop 规划多步。
5. 验收：唤醒→"前进半米"端到端 < 2s；断 WiFi 时机器人 0.5s 内停。

---

## 方向 2：VLA + 语音融合（底盘移动）

### 为什么不选 Alpamayo

NVIDIA Alpamayo(-R1) 是**自动驾驶** VLA：输入多路车规相机+自车运动学，
输出道路驾驶轨迹，语义空间是车道/交规/行人。室内麦轮小车与它 domain、
传感器、动作空间三重不匹配，微调成本远大于收益。

### 推荐：分层架构（VLM 规划 + 传统执行），端到端 VLA 留给机械臂

- **底盘移动**本质是导航问题。用 **VLM（Qwen2.5-VL @ vLLM，内网 GPU）**
  做"眼睛和脑子"：开放词汇目标 grounding（"有红色杯子的桌子"→ 图像里
  框出 → 反投影到地图坐标）+ 任务分解；**Nav2 做"腿"**。这条路不需要
  采数据、不需要训练、可靠性和可解释性都比端到端 VLA 高一个量级。
- 端到端 VLA（pi0/SmolVLA/GR00T N1.5）的价值在**接触型操作**（抓取/
  开门），等机械臂（方向 3）上了再用，且 lerobot 生态数据采集/微调
  工具链现成。

### 实施步骤

1. **Nav2 接入**：AMCL 换成现有 loc 模式（map→odom 已有），配
   costmap（`/scan` + `/rtabmap/grid_map`）、DWB/MPPI 控制器（麦轮全向
   底盘用 omni 运动学）。
2. **语义地点表**：SLAM 地图上标注命名点位（先手动 yaml：厨房/沙发/
   充电点 → map 系坐标），`robot.goto(place)` 查表发 Nav2 goal。
3. **VLM grounding 服务**（服务器）：订阅
   `/camera/color/image_raw/compressed` + 深度 + 当前位姿，暴露
   `find_object(text) → map 坐标`；结合 depth 反投影 + loc 位姿转 map 系。
4. **AIRE Agent 打通**："去有红色杯子的桌子" → Agent 调 `find_object`
   → `robot.goto(坐标)` → 到达后 VLM 确认 → TTS 汇报。
5. （可选，实验性）想玩端到端：SmolVLA 加 mobile-base 动作头，用手柄
   遥操作录 lerobot 格式数据微调——但建议在分层方案跑通后再投入。

算力/延迟评算：VLM 7B 单张 4090 即可（<1s/次调用，规划频率低）；图像
上行走已有 compressed 流 ~1Mbps；Nav2 全在 Pi 本地闭环，跨网只传 goal。

---

## 方向 3：机械臂（lerobot SO-101）

### 硬件路线（关键决策）

- **臂选 SO-101**（lerobot 标准臂）：6× Feetech STS3215 总线舵机，
  结构件 3D 打印开源，BOM ≈ ¥700-1000/臂。建议同时买 **leader 臂**
  （再加一套减配版）用于遥操作采数据——VLA/ACT 微调离不开它。
- **STM32 控制算法：不需要，也不建议做。**
  - RRCLite 的总线舵机口（Function=5）是 Hiwonder 私有协议（LX 系列），
    与 Feetech STS3215 协议不兼容；
  - lerobot 的采数据/回放/推理栈原生走 USB-TTL 适配器直连 Feetech 总线
    （`FeetechMotorsBus`），走 STM32 = 自研固件透传 + 断掉整个生态；
  - 分工：STM32 继续管底盘电机/云台/蜂鸣器；臂走
    **Feetech USB 适配器 → Pi USB 口**，软件即插即用。
- 舵机内置编码器+位置控制，"控制算法"就是 lerobot 侧 30-50Hz 位置流 +
  软件限位/限流，无需自己写伺服环。

### URDF：需要

- `mentorpi_description` 已有 mecanum.xacro（本仓库）。SO-101 官方有
  URDF/MJCF：做一个 `so101.xacro` include，挂到 `base_link` 上
  （`arm_base_joint`，位姿 = 转接板实测值）。
- 用途：TF 一致性（臂末端在 map 系的位姿）、RViz/Foxglove 可视化、
  MoveIt2（可选 IK）、仿真采数据（MuJoCo/Isaac）、VLA 训练时的
  proprioception 对齐。

### 3D 打印底座接入

- 设计**转接板**：上层匹配 SO-101 底座孔位，下层匹配 MentorPi 顶板
  孔位（量现车）。PETG ≥40% 填充，或 3mm 碳板更稳。
- **翻覆是主要风险**：底盘轮距仅 13.7×14.1cm，SO-101 臂展 ~35cm、
  自重 ~1kg。对策：臂座靠底盘几何中心偏后、电池前移配重、软件限制
  工作空间（限位 + 满伸展降速）、抓取时底盘锁死（cmd_vel 抑制）。
- 相机视野：检查臂 home 位不挡云台相机；操作任务建议臂上加一个
  腕部相机（lerobot 常规配置，USB 直连 Pi）。

### 机械数模、打印件与 URDF 回归

- 建立 `mechanical/` 目录作为机械设计源，保存原生参数化 CAD/STEP、实测尺寸、
  装配基准和版本说明；现有 STL 只作为外形参考，不反向当作精确参数化模型。
- 优先获取 Gemini 2L、MS200、SO-101、电池、麦克风和 USB Hub 的厂商 STEP；
  缺失时按实物重建安装面、孔位和包络，先校验 STL 比例与关键尺寸。
- 建立整机 CAD 装配：底盘基准 → 安装孔 → 支架 → 外挂件；视觉标定得到的
  光学坐标外参不能替代机械孔位和安装面尺寸。
- 打印件以原生 CAD 为源，输出 3MF/STL，并记录材料、方向、壁厚、填充率、
  热熔螺母和公差；装配后进行干涉、线束、视野、重心和翻覆检查。
- CAD 定型后导出简化 visual mesh 和独立 collision mesh，加入模块化 xacro；
  实测质量、质心和必要的惯量后再用于 MoveIt、MuJoCo、Gazebo/Isaac 仿真。
- 推荐目录：
  `mechanical/assemblies`、`mechanical/references`、
  `mechanical/accessories`、`mechanical/printable`、
  `mechanical/measurements`。STEP/原生 CAD 是机械源，xacro 是 ROS
  结构与运动学源，两者通过明确命名的安装基准同步。

### 供电与线束

- STS3215 供电 6-7.4V，6 舵机峰值电流 5A+。**独立供电，不从 RRCLite
  舵机口或 Pi 5V 取电**：
  - 底盘电池（确认电压，通常 2S 7.4V）→ 独立 DC-DC/直连 + **5A 保险丝**
    + 独立开关 → Feetech 总线电源端；
  - 与 Pi **共地**（USB 适配器信号地）；
  - 总电流预算：4 底盘电机 + 6 舵机 + Pi5(5V/5A) + 相机 + 雷达，峰值
    可到 15A@7.4V 量级——确认电池放电 C 数和 BMS 限流，必要时臂用
    独立电池（最简单、隔离最好）。
- 线束：舵机菊花链走臂内槽（SO-101 打印件自带），电源线 18AWG 硅胶线，
  留服务环避免关节拉扯。

### 实施步骤

1. 硬件：打印+组装 SO-101（follower + leader），转接板设计打印，供电
   改造；Feetech USB 适配器插 Pi。
2. lerobot 直连跑通：Pi 上装 lerobot，标定舵机 ID/零位，leader→follower
   遥操作。
3. URDF 合并 + TF：so101.xacro 挂 base_link，joint_states 桥接
   （lerobot 读的位置 → `/joint_states` → robot_state_publisher）。
4. 数据采集：腕部相机 + 云台相机双视角，每任务 50-200 episodes
   （lerobot 格式）。
5. 策略训练（内网 GPU）：先 ACT（小、稳、可在 Pi 上 ONNX 30Hz 推理），
   再 SmolVLA/pi0 微调（服务器推理 + action chunk 下发）。
6. 语音融合闭环：AIRE Agent 编排 `robot.goto(桌子)` →
   `arm.run_task("pick_cup")` → VLM 验证 → TTS 汇报。

---

## 分阶段总排期（建议顺序）

| 阶段 | 内容 | 依赖 |
|------|------|------|
| P0 | TODO.md 标定 Part 1-3 + Nav2 接入 + motion_primitive 节点 | 无 |
| P1 | AIRE robot skill + Pi 麦克风/薄边缘端 + 快慢双路语音控制 | P0(部分) |
| P2 | 语义地点表 + VLM grounding + "语音→导航"闭环 | P0 Nav2 |
| P3 | SO-101 硬件 + lerobot 遥操作 + URDF 合并 | 可与 P1/P2 并行 |
| P4 | 操作数据采集 + ACT/SmolVLA 训练 + 语音编排长程任务 | P2 + P3 |

## 主要风险

1. **Pi CPU 余量**：slam_3d + 语音边缘端逼近上限 → 语音降级为服务器
   全托管（麦克风裸流直推），或运行时互斥（导航时降相机帧率）。
2. **WiFi 断连**：安全层已在本地（watchdog/deadman/Nav2 本地避障），
   跨网只传意图——架构上已规避，测试时要专门演练断网。
3. **底盘翻覆/打滑**（带臂后质心升高）：限工作空间 + 操作时锁底盘 +
   麦轮参数重标定（负载变了，TODO Part 1 要重跑）。
4. **VLA 数据成本**：每个操作任务 50+ episodes 遥操作，先用 ACT 单任务
   验证全链路，再谈通用策略。
