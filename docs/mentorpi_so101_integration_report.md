# MentorPi × LeRobot SO-101 集成实施与验证报告

- 日期：2026-07-18
- 状态：首版概念设计与软件/零位几何验证完成，等待尺寸复核、实物装配和带载测试
- 范围：机械安装草案、电源架构建议、USB 拓扑、URDF/网格集成、独立零件制造文件

## 1. 项目目的

本次工作的目标，是在不破坏 MentorPi 现有底盘、相机标定和基础 URDF
行为的前提下，为 LeRobot SO-101 follower 机械臂提供一套可落地的车载集成方案。
具体要求如下：

1. 评估底盘网格中的候选安装特征，设计 SO-101 可打印转接件。
2. 利用现有双 USB-C Anker Prime 充电宝设计独立机械臂供电方案和低位固定件。
3. 利用现有小米 USB Hub 连接树莓派、Feetech 控制板和低带宽外设。
4. 输出新的 MentorPi + SO-101 URDF、SO-101 网格和打印用 STL。
5. 以低位布局降低新增负载重心，并评估相机和激光雷达的几何间隙。
6. 保持未启用机械臂时的原 MentorPi 模型和运行时模式兼容。

本次不是路线图“方向 3”的完整实施。尚未完成实物打印和装车、leader/follower
组装与标定、舵机 ID/零位配置、LeRobot 直连遥操作、`/joint_states` 桥接、
MoveIt 或控制联锁、腕部相机、数据采集和训练。

## 2. 输入、依据与边界

### 2.1 仓库内依据

- `docs/roadmap.md` 中“方向 3：机械臂”的初步规划。
- `src/mentorpi_description/urdf/mecanum.xacro` 中的底盘、相机和雷达坐标。
- `src/mentorpi_description/meshes/mecanum/base_link.STL` 的仓库参考网格。
- `docs/power_troubleshooting.md` 和 `docs/hardware_protocol.md` 中已有供电及 USB
  带宽问题记录。
- 当前标定相机变换保持不变；机械臂方案不得以移动相机为代价。

### 2.2 外部依据

- SO-101 几何、关节、质量和网格取自
  [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
  的提交 `fda892c`，采用其 new-calibration URDF。
- 控制与总线结构参考
  [LeRobot SO-101 文档](https://huggingface.co/docs/lerobot/en/so101)。
- 舵机电压边界参考
  [Feetech STS3215 数据表](https://www.feetechrc.com/Data/feetechrc/upload/file/20200611/6372749961523760249976542.pdf)。
- 充电宝尺寸和输出档位按
  [Anker Prime A1335 官方规格](https://service.anker.com/product-description/a085g00000Giu9GAAR/anker-prime-12000mah-power-bank130w%3Fref%3DHome_Page)。
- Pi 5 的供电限制参考
  [Raspberry Pi 官方文档](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)。

### 2.3 已知边界

本次结果是完成几何和软件验证的首版工程方案，不是结构强度、电池安全或电气
合规认证。当前没有对真实车体进行游标卡尺复测、满载急停、坡面稳定性、舵机堵转
或长时间温升试验，也没有整机质量台账、合成质心、翻覆角或制动惯性计算。

## 3. 方案概述

### 3.1 总体布局

- 机械臂转接板按参考 STL 中识别的四个候选安装特征布局，中心为
  `base_link xyz = -0.0409, 0, 0.0655 m`，绕 Z 轴旋转 180°。
- 候选特征间距约为 `20.1 × 48.7 mm`。其中 x≈-50.9 mm 的两个 Ø4.3 mm
  特征在网格中只有约 1.5 mm 深，可能是浅孔或沉孔；x≈-30.9 mm 的两个特征才
  表现为贯穿结构。四点能否用于攻丝或贯穿固定必须拆机确认。
- 充电宝横置于车头低位，作为前部配重的布局意图；重心改善尚未量化验证。
- 原有相机安装位和标定变换不变。
- 启用 SO-101 时，激光雷达安装面提高 170 mm；不启用机械臂时保持原高度。
- 小米 Hub 目前只有不依赖外壳尺寸的绑带托盘草案，尚未确定装车坐标和固定接口。

### 3.2 供电方案

推荐的机械臂独立电源支路为：

```text
Anker USB-C1
  -> 20 V USB PD sink/诱骗板
  -> 7.5 V、器件额定输出能力不低于约 8 A 的同步降压模块
  -> 7.5 A 保险丝
  -> 带锁止机械臂电源开关
  -> Feetech 控制板电源输入
  -> 6 × STS3215
```

设计要点：

- USB 只承担 Feetech 控制数据和信号地，不能给整条舵机总线供电。
- “额定 8 A”用于给降压器本身留裕量，不表示系统可持续输出 8 A。A1335 单口
  20 V/3.25 A 上限约 65 W；实际 7.5 V 连续输出还受转换效率、线缆和温升限制，
  必须台架实测。
- 超过 3 A 的 PD 档位需要合适的 E-marker USB-C 线缆和额定连接器。
- 不从树莓派 USB、Pi 5 V 轨或 RRCLite 私有总线舵机口给 SO-101 供电。
- 7.5 V 支路初选 18 AWG 硅胶线和 7.5 A 保险丝；最终线径、保险丝额定值和
  保护曲线应根据台架峰值、持续电流、连接器和线束温升确认。
- 第一次接舵机前，先空载设定并实测输出电压、极性和限流。
- Anker A1335 的 5 V/3 A 档位不能替代 Pi 5 满规格 5 V/5 A 电源；Pi 继续使用
  现有 RRCLite 电源，台架测试可使用官方 27 W 电源。
- 仅当小米 Hub 明确具有带反灌隔离的 PD-in 时，才允许由第二个 USB-C 口给 Hub
  供电；否则 Hub 只作为总线供电的数据扩展。

### 3.3 USB 拓扑

```text
Raspberry Pi 蓝色 USB 3
  -> Gemini 2 深度相机（保持直连）

Raspberry Pi 另一 USB
  -> 小米 Hub
      -> Feetech USB 控制板
      -> 键鼠接收器等低带宽设备
```

相机保持直连，是因为仓库已有该相机经过普通 Hub 后双流带宽明显下降的记录。

## 4. 交付产物

### 4.1 参数化 CAD、STEP 与 STL

STEP 是机械设计主交付，STL 是打印派生物。所有打印件均保留可重新生成的 Python
源文件。

| 零件 | 作用 | 关键尺寸/特征 | 源文件与输出 |
|---|---|---|---|
| SO-101 转接板 | 底盘四个候选安装特征到官方 SO-101 夹具平台 | 118 × 86 × 8 mm；M4 通孔、沉孔、4 个 20 mm 绑带槽 | `mechanical/printable/arm_adapter_plate.py`、`.step`、`stl/arm_adapter_plate.stl` |
| Anker Prime 前托架 | 低位固定充电宝并形成前配重 | 按 A1335 包络 134.4 × 55 × 34.53 mm；双绑带槽、开口 USB 端 | `mechanical/printable/anker_prime_front_tray.py`、`.step`、`stl/anker_prime_front_tray.stl` |
| 雷达增高座 | 将扫描面抬高至机械臂零位姿态上方 | 底板 62 × 54 × 5 mm；总升高 170 mm | `mechanical/printable/lidar_riser.py`、`.step`、`stl/lidar_riser.stl` |
| 小米 Hub 托架草案 | 尚未定位、不依赖具体型号的绑带托盘 | 115 × 38 × 13 mm；未采用真实 Hub 包络 | `mechanical/printable/xiaomi_hub_carrier.py`、`.step`、`stl/xiaomi_hub_carrier.stl` |

ROS 使用的四个打印件副本位于
`src/mentorpi_description/meshes/accessories/`。

当前没有完整的整机 STEP 装配，也没有给 PD sink、降压器、保险丝、开关和
Feetech 板设计固定壳或装车坐标。URDF 仅作为第一版布局代理；线束路径、装配级
干涉、动态扫掠和整机重心尚未验证。

### 4.2 SO-101 与 URDF 产物

- `src/mentorpi_description/urdf/so101.xacro`：由上游固定提交确定性生成，
  所有 link/joint/material 名称添加 `so101_` 前缀。
- `src/mentorpi_description/urdf/mentorpi_so101.urdf`：可直接检查或交给查看器的
  完整展开 URDF。
- `src/mentorpi_description/meshes/so101/`：13 个上游 SO-101 STL 网格。
  发布或再分发前仍需补充来源清单、校验值，并复核上游许可证义务。
- `mechanical/urdf/build_so101_xacro.py`：从固定上游 URDF 重建 xacro。
- `mechanical/urdf/gen_mentorpi_so101.py`：从项目 xacro 重建最终 URDF。
- `mechanical/urdf/design-ledger.md`：坐标、单位、上游版本和假设记录。

对现有包的改动：

- `mecanum.xacro` 新增 `with_so101` 参数；默认 `false`。
- `display.launch.py` 可通过 `with_so101:=true` 预览完整模型。
- `setup.py` 安装 SO-101 和附件网格目录。
- `with_so101:=false` 时仍使用原雷达高度和原底盘树。

### 4.3 制造快照与说明

- 六张 CAD 视图快照位于 `mechanical/printable/snapshots/`。
- 打印参数、接线、装配和调试流程位于 `mechanical/README.md`。
- 底盘孔位分析脚本为 `mechanical/measurements/analyze_base_mesh.py`。
- 零位姿态间隙检查脚本为
  `mechanical/measurements/check_urdf_clearances.py`。

## 5. 验证过程与结果

### 5.1 CAD 生成和几何检查

四个模型均完成：

1. Python 参数化源生成。
2. STEP 主模型导出。
3. STL 打印网格导出。
4. 几何实体、包围盒、平面和定位检查。
5. 等轴测或顶视图快照检查。

这些检查针对四个独立零件，不是整机装配干涉检查。当前仓库没有保存 CAD inspect
过程的机器可读日志；下表是本次会话中核对的外包络摘要。

最终外包络：

| 零件 | 外包络 |
|---|---|
| 转接板 | 118 × 86 × 8 mm |
| Anker 托架 | 50.13 × 142 × 58 mm |
| 雷达增高座 | 62 × 54 × 175 mm |
| Hub 托架 | 115 × 38 × 13 mm |

### 5.2 URDF 解析与回归检查

完成以下三种配置的 xacro 展开和 `check_urdf`：

```bash
# 完整 MentorPi + SO-101
check_urdf src/mentorpi_description/urdf/mentorpi_so101.urdf

# 原底盘回归
xacro src/mentorpi_description/urdf/mentorpi.xacro \
  with_so101:=false -o /tmp/final_base.urdf
check_urdf /tmp/final_base.urdf

# runtime_mode + SO-101
xacro src/mentorpi_description/urdf/mentorpi.xacro \
  runtime_mode:=true with_so101:=true -o /tmp/final_runtime_arm.urdf
check_urdf /tmp/final_runtime_arm.urdf
```

三种配置均成功解析：

- 标准完整模型根节点为 `base_footprint`。
- runtime 模型根节点为 `base_link`。
- SO-101 六个运动关节和 gripper frame 位于唯一、连通的树中。
- 原底盘配置不含任何新增机械臂或附件 link。

### 5.3 零位姿态 AABB 高度与扫描平面检查

运行：

```bash
/tmp/mentorpi-cad-venv/bin/python \
  mechanical/measurements/check_urdf_clearances.py \
  src/mentorpi_description/urdf/mentorpi_so101.urdf
```

结果：

```text
camera_link       z = 0.1161 .. 0.1481 m
anker_prime_link  z = 0.0535 .. 0.1085 m
battery_to_camera_vertical_clearance_mm = 7.6

laser_scan_z_m = 0.4005
so101_zero_pose_max_z_m = 0.3817
so101_links_crossing_scan_plane = []
```

在该脚本的限定范围内可知：

- 充电宝 visual AABB 顶部比相机 visual AABB 底部低 7.6 mm。
- 零位姿态下没有 SO-101 link 穿过雷达扫描平面。
- 雷达扫描面比机械臂零位姿态最高点高 18.8 mm。

该检查没有建立 Gemini 2 的彩色/深度视锥，也没有验证机械臂、雷达柱、Hub 或
充电宝是否遮挡真实图像下沿。因此“相机无遮挡”尚未验证，必须在实机彩色和深度
画面中复核。该检查同样不等同于全关节空间扫掠体分析；运行机械臂时仍需将雷达
避障、自碰撞和底盘联锁加入控制策略。

### 5.4 本次临时环境中的可重复生成、构建和测试

执行并通过：

```bash
# SO-101 xacro 可重复生成
python3 mechanical/urdf/build_so101_xacro.py \
  /tmp/SO-ARM100/Simulation/SO101/so101_new_calib.urdf \
  /tmp/so101_rebuilt.xacro
cmp -s /tmp/so101_rebuilt.xacro \
  src/mentorpi_description/urdf/so101.xacro

# ROS 构建
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select mentorpi_description

# ROS 包测试
source install/setup.zsh
colcon test --packages-select mentorpi_description
colcon test-result --verbose
```

生成过程依赖仓库外的 `/tmp/SO-ARM100` 固定提交，以及
`/tmp/mentorpi-cad-venv` 中的 build123d/cadpy 等依赖；孔位分析还依赖
`scikit-learn`。仓库尚未包含上游输入下载与 SHA256 校验脚本、锁定依赖文件或
永久构建环境，因此这里只能证明本次临时环境内可重复，不能证明全新机器上开箱即复现。

最终结果：

```text
Build: 1 package finished
Tests: 2 passed, 1 skipped, 0 errors, 0 failures
```

跳过项为模板中的 copyright header 检查，不影响功能。另已通过：

- 新增及修改 Python 文件的 `python3 -m py_compile`。
- `git diff --check`。
- 四个 STEP、四个打印 STL、13 个 SO-101 STL 和最终 URDF 的存在性检查。
- 本地 CAD Viewer HTTP 服务响应检查。

Viewer 服务响应只证明临时 HTTP 会话可访问；六张 PNG 快照承担静态人工外观复核，
两者均不替代完整装配渲染。未运行 RViz GUI、实时 `robot_state_publisher` 树、
MoveIt、Gazebo 或 MuJoCo consumer smoke test。

## 6. 使用方法

不启用机械臂，保持原模型：

```bash
xacro src/mentorpi_description/urdf/mentorpi.xacro \
  with_so101:=false -o /tmp/mentorpi.urdf
```

启用机械臂和配套附件：

```bash
xacro src/mentorpi_description/urdf/mentorpi.xacro \
  with_so101:=true -o /tmp/mentorpi_so101.urdf
```

RViz 预览：

```bash
ros2 launch mentorpi_description display.launch.py with_so101:=true
```

## 7. 实物装配前必须完成的检查

1. 拆机确认四个候选特征是否为孔、螺纹孔或沉孔，并用卡尺确认中心、孔径、有效
   深度、顶板厚度和前唇厚度；不得在确认前按四个贯穿孔采购紧固件。
2. 确认充电宝底部标签型号；若不是 A1335，修改 `PACK_*` 常量后重新生成。
3. 先打印孔位量规或转接板，禁止通过强拧螺丝修正孔位误差。
4. 充电宝托架必须同时使用两条绑带，不能只依赖打印挂钩。
5. 检查充电宝 USB 端开口、轮胎扫掠空间和至少约 40 mm 离地间隙。
6. 安装雷达后重新测量真实扫描中心高度，并复核水平度。
7. 确认小米 Hub 的确切型号；没有明确反灌隔离资料时不得外接 PD-in。
8. 在只接一个舵机的情况下验证 7.5 V 电源，再逐步接入完整菊花链。
9. 进行急停、堵转峰值、电压跌落、线缆和降压模块温升试验。
10. 进行折叠姿态、全关节低速扫掠和底盘制动稳定性测试。
11. 加载完整负载后重新做底盘运动学和定位标定。
12. 用实机彩色与深度流检查整个视场，尤其是画面下沿遮挡。
13. 建立整机质量台账，计算合成质心、支撑多边形和满伸展翻覆裕度。

## 8. 风险与后续工作

| 风险/未决项 | 当前控制措施 | 后续动作 |
|---|---|---|
| 四个 STL 特征并非都已证明为贯穿孔 | M4 间隙孔、仅作为候选布局 | 拆机识别浅孔/沉孔/螺纹并卡尺复测 |
| 充电宝型号可能不是 A1335 | 参数化 PACK 常量 | 核对底部型号和三维尺寸 |
| 机械臂峰值电流与 65 W PD 实际余量未知 | 独立支路、额定高电流降压器初选 | 堵转、动态峰值、线损和温升记录后确定保险丝 |
| 170 mm 雷达柱振动 | PETG-CF、较高壁数和填充 | 实车测振，必要时加斜撑 |
| 零位检查不覆盖全工作空间 | 仅声明零位通过 | 增加关节空间扫掠和碰撞矩阵 |
| 相机视锥未建模 | 只确认 visual AABB 高度分离 | 实机检查彩色/深度视场下沿 |
| 低位布局没有量化 CG 或翻覆裕度 | 电池横置低位作为启发式措施 | 质量台账、合成质心、满伸展和急停测试 |
| 前部 360 g 配重改变底盘参数 | 低位横置 | 重做加速度、制动和里程计标定 |
| Hub 供电/反灌能力不明 | 默认只做数据 Hub | 查明准确型号与电路规格 |
| Hub 托盘及电源组件没有装车位置 | 仅交付独立托盘和电气架构 | 取得实物尺寸后完成整机 STEP 装配与固定壳 |
| 上游和 CAD 构建依赖未锁定 | 记录提交和临时环境 | 增加 provenance、校验值和依赖锁文件 |
| 打印件惯量为工程估算 | URDF 中明确为初版 | 称重并更新质量、质心和惯量 |

## 9. 工作区说明

报告生成时，`src/mentorpi_supervisor/foxglove_layout/loc_check.json` 已存在与本任务
无关的用户修改。本次工作未修改、恢复或覆盖该文件。

## 10. 审阅状态

真正的 Fable 尚未审阅本报告。当前版本已准备好提交给 Fable，但不得将下面的
内部复核记录署名为 Fable。

2026-07-18，Codex 在当前工作区内完成了一次只读、证据导向的内部交叉复核。

Codex 内部复核通过：

- 最终 `mentorpi_so101.urdf` 可由 `check_urdf` 解析。
- 间隙脚本复现 7.6 mm、0.4005 m、0.3817 m 和零位 `crossing=[]`。
- `git diff --check` 通过。
- 4 个 STEP、4 个打印 STL、6 张 PNG 和 13 个 SO-101 STL 均存在。
- pytest XML 为 2 passed、1 skipped。

根据内部复核意见，本报告已修正或明确披露：候选孔并非四个已确认贯穿孔、相机视锥
未验证、低重心未量化、65 W PD 对实际输出电流的限制、临时环境依赖、缺少整机
STEP 装配、Hub 和电源组件未定位，以及路线图后续控制和实物验证尚未完成。
