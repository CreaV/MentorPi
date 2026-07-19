# MentorPi + SO-101 当前交接

- 更新日期：2026-07-19
- 当前分支：`feat/voice-vla-extensions`
- 几何修正基线提交：`fdd8c1f fix(description): restore lidar geometry and SO-101 deck`
- 推送状态：几何修正基线尚未 push；准确 ahead 数以会话开始时的 `git status --short --branch` 为准
- 用户工作区：`src/mentorpi_supervisor/foxglove_layout/loc_check.json` 是用户原有未提交修改，禁止顺手暂存、覆盖或回退

这份文件只描述当前可继续工作的状态，不是历史调查日志。若它与代码或重新运行的验证结果冲突，以参数化源文件和验证结果为准，并直接更新本文件，不要在旧结论上继续追加更正块。

## 1. 先读结论

- SO-101 朝前安装，`base_link -> so101_base_link` 为 `xyz=-0.155 0 0.0655`、`rpy=0 0 0`。
- MS200 直接贴装在底盘顶板，没有塔架、立柱或增高座。`base_link -> laser_frame` 为 `xyz=-0.012242 0 0.092501`、`rpy=0 0 0`。
- `base_link` 离地 50.5 mm；底盘 STL 顶面和雷达 STL 底面都在离地 108.001 mm；扫描面离地 143.001 mm。
- 雷达后方真实孔型是“前 2、后 4”。甲板只使用后排四孔：底盘坐标 `x=-61 mm`，`y=-24/-8/+8/+24 mm`，相邻中心距 16 mm，STL 名义孔径约 Ø4.3 mm。
- 用户确认这些孔无螺纹、贯穿、与雷达固定无关。甲板使用 Ø4.5 mm 通孔，以 M4 螺丝、垫圈和螺母固定；前两个孔不用。
- 甲板范围是底盘坐标 `x=-195..-50 mm`、`y=±48 mm`，厚 8 mm；前缘与雷达 STL 后界保留 10.95 mm 平面间隙，后悬伸仍为 92 mm。
- 恢复雷达 TF 后，机械臂 `shoulder_pan` 全 ±110° 扫转到雷达头的最小网格间隙是 36.2 mm（-19° 附近）。
- 真实 STL 与扫描面的交线给出后向几何半宽 18.11°；加 5° 安全余量并向上取整，当前掩膜为 ±24°（0.418879 rad），不是旧的 ±52°。
- 相机 2-DOF 云台已经拆除。`camera_joint` 中的旧外参只保留作来源记录，当前无效；最终相机支架定型前不要重新标定，也不要为改善渲染效果修改外参。

## 2. 当前权威文件

| 内容 | 参数化源 / 权威输入 | 派生物或运行配置 |
|---|---|---|
| 底盘、雷达、相机、SO-101 装配 | `src/mentorpi_description/urdf/mecanum.xacro`、`so101.xacro` | `mentorpi_so101.urdf`、`mentorpi_so101.viewer.urdf` |
| 雷达恢复依据 | `src/mentorpi_description/urdf/mecanum.pre_calibration.xacro` + `meshes/mecanum/*.STL` | `laser_joint` 与掩膜计算输入 |
| SO-101 甲板 | `mechanical/printable/so101_deck_plate.py` | `.step`、`mechanical/printable/stl/so101_deck_plate.stl`、包内 accessories STL |
| 扫转间隙 | `mechanical/measurements/check_so101_clearances.py` | 控制台验证结果 |
| 扫描掩膜 | `mechanical/measurements/compute_scan_mask_so101.py` | `src/mentorpi_bringup/config/scan_mask_so101.yaml` |
| 整机零位预览 | 正式 URDF + `mechanical/urdf/bake_urdf_glb.py` | gitignore 的 `mentorpi_so101.preview.glb` |
| 设计摘要 | `mechanical/README.md`、`mechanical/urdf/design-ledger.md` | 本交接文档 |

编辑源文件后必须重新生成派生物。不要直接手改 STEP、STL 或生成后的 URDF 来制造“看起来正确”的结果。

## 3. 已完成与验证证据

提交 `fdd8c1f` 已完成：

- 恢复雷达直装 TF，删除虚构雷达塔架和已拆除相机云台的装饰件。
- 重做甲板为后排四孔方案，重新生成 STEP/STL 并同步 ROS 包副本。
- 新增可复现的扫描面/网格交线脚本，更新掩膜为 ±24°。
- 重新生成正式 URDF、viewer URDF 和零位 GLB。
- 更新标定、硬件、机械设计文档，并明确相机旧外参失效。

本轮已实际通过：

- CAD 确定性检查：单零件，包围盒 145×96×23 mm；四个甲板孔 Ø4.5 mm；三段相邻孔距均为 16.0 mm。
- CAD 等轴测与底视快照：板面、孔、绑带槽、双纵肋和后横 web 完整。
- 间隙脚本：最差雷达头间隙 36.2 mm。
- 掩膜脚本：几何半宽 18.11°，建议 ±24°，边界 `[2.722714, 3.560472]` rad。
- `check_urdf`：正式 URDF 和 viewer URDF 均解析成功。
- `with_so101:=false`：只保留底盘、轮、IMU、相机、雷达，不混入任何 SO-101 链接。
- `colcon build --packages-select mentorpi_description`：通过。
- `git diff --check`：通过。

## 4. 下一步，按这个顺序

### A. 甲板实物干装

不再测雷达高度或重新判断孔型。只需要：

1. 确认四孔下方有足够空间放垫圈和螺母。
2. 用甲板或小孔规干装，按“底盘板厚 + 8 mm 甲板 + 垫圈 + 螺母”选择 M4 螺丝长度。
3. 确认不会压住线束、接插件或底盘内部零件后，才进入正式切片。

### B. 相机最终安装与重标

1. 先确定永久支架和相机朝向，确保紧固后不会移动。
2. 再按 `docs/calibration.md` 采集新的 AprilTag 数据并更新 `camera_joint`。
3. 重建 description 包，检查地面点云和视野下沿；不要用 URDF 截图判断外参正确性。

### C. 前置 2S 电池托架

甲板不再阻塞托架，但托架仍需两个实物输入：

- 中层甲板厚度，写入 `battery_tray_2s.py` 的 `HOOK_THROAT`。
- 购买的 2S 电池长/深/高，写入 `PACK_LENGTH_Y`、`PACK_DEPTH_X`、`PACK_HEIGHT_Z`。

改常量后重新生成 STEP/STL，再做干装和轮胎/离地间隙检查。

### D. 部署和装车验证

1. Pi 安装 `ros-jazzy-laser-filters`。
2. 构建 `mentorpi_description mentorpi_bringup mentorpi_supervisor`，source 安装空间。
3. 用 `ros2 launch mentorpi_supervisor remote.launch.py with_so101:=true` 启动；systemd 的 `ExecStart` 也要传同一参数。
4. 确认雷达驱动发 `/scan_raw`、滤波器发 `/scan`，下游仍只订阅 `/scan`。
5. 收拢机械臂，实测 ±24° 掩膜能消除自体回波，同时检查后向真实障碍盲区。
6. 带载重新跑底盘标定/验收，再重扫 2D/3D 地图。旧 3D 数据受相机移位影响应视为失效；旧 2D 图即使能用也建议重扫。

### E. 后续阶段

- 2S + 保险丝 + 带锁开关到 Feetech 板的电气台架验证。
- SO-101 舵机 ID、零位、限位和 LeRobot 接入。
- 折叠运输姿态、停车操作约束、速度/力矩上限。
- 如进入 MoveIt，再建立 SRDF、碰撞矩阵和规划组。

## 5. 再生成与复核顺序

1. 先编辑 xacro 或 Python CAD 源。
2. 甲板用 CAD 工具从 `so101_deck_plate.py` 再生成 STEP 和 STL；确定性检查后，把 STL 同步到 `src/mentorpi_description/meshes/accessories/`。
3. `colcon build --packages-select mentorpi_description`，避免顶层 xacro 的 `$(find mentorpi_description)` 读到旧安装副本。
4. 用 `mechanical/urdf/gen_mentorpi_so101.py` 和 `gen_mentorpi_so101_viewer.py` 再生成显式 URDF。
5. 对两份 URDF 运行 `check_urdf`，并额外生成一次 `with_so101:=false` 做回归。
6. 运行间隙脚本和掩膜脚本；如果结果改变，同步更新 YAML、README、design ledger 和本交接。
7. 烘焙 GLB 做整机视觉复核。当前 three.js 系 URDF viewer 会把 fixed-joint 子树散开，不能拿它判断装配位置；静态 GLB 才是可靠预览。

常用验证：

```bash
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select mentorpi_description
source install/setup.zsh

check_urdf src/mentorpi_description/urdf/mentorpi_so101.urdf
check_urdf src/mentorpi_description/urdf/mentorpi_so101.viewer.urdf

/tmp/mentorpi-cad-venv/bin/python \
  mechanical/measurements/check_so101_clearances.py \
  src/mentorpi_description/urdf/mentorpi_so101.urdf

/tmp/mentorpi-cad-venv/bin/python \
  mechanical/measurements/compute_scan_mask_so101.py \
  src/mentorpi_description/urdf/mentorpi_so101.urdf
```

`/tmp/mentorpi-cad-venv` 是临时环境，不存在时应按 CAD skill 重建，不要把它当仓库依赖。

## 6. 禁止回退的错误假设

- 不得恢复 `laser_joint z=0.18`。
- 不得重新添加雷达塔架、四立柱或 170 mm 增高座。
- 不得把后排四孔当成雷达固定孔，也不得让甲板压到雷达下面。
- 不得恢复旧 ±52° 掩膜，除非新的真实装配和重新计算明确要求。
- 不得把已拆云台的相机外参当成当前有效值。
- 不得为了让渲染图“居中”而修改标定 joint；视觉 mesh 的内部偏移与外参是两件事。
- 不得只改生成物而不改源文件。
- 不得在没有检查 `git status` 的情况下覆盖或提交用户的 `loc_check.json`。

## 7. 开始下一次会话时

```bash
git status --short --branch
git log -3 --oneline --decorate
```

然后按顺序读：

1. 本文件。
2. `mechanical/README.md` 顶部的 layout v3 当前状态。
3. `mechanical/urdf/design-ledger.md` 的 layout v3。
4. 计划修改的参数化源文件。

不要先读旧提交日志来反推当前状态；历史只在需要解释某个决定时再查。
