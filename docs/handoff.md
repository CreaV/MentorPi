# Agent Handoff

- 更新时间：2026-07-21
- 分支：`feat/voice-vla-extensions`
- 当前基线：`503e037 docs: replace SO-101 handoff with current-state guide`
- 会话开始时 `HEAD` 与 `origin/feat/voice-vla-extensions` 同步
- 用户自有工作区修改：`src/mentorpi_supervisor/foxglove_layout/loc_check.json`，不要覆盖、回退或顺手提交

本文是整个仓库唯一的滚动会话交接，不绑定某个 feature。下一次 agent 先读本文件；下一轮结束时应根据当轮对话重新压缩、核对并覆盖本文。

## 本轮主要做了什么

本轮围绕 SO-101 车载集成完成了从错误假设清理到可制造 CAD/URDF 的闭环：

1. 阅读旧交接和恢复的 `mecanum.pre_calibration.xacro`，用底盘、雷达 STL 复核雷达实际位置。
2. 用户确认底盘雷达后方孔型是“前 2、后 4”；后排四孔无螺纹、贯穿且不参与雷达固定，并同意使用 STL 尺寸作为测量值。
3. 恢复 MS200 直装 TF，删除虚构雷达塔架；重做 SO-101 甲板为后排四孔方案。
4. 重新生成 STEP、STL、正式 URDF、viewer URDF、零位 GLB和扫描掩膜。
5. 增加可复现的机械臂间隙与扫描面交线脚本，完成 CAD 快照、URDF、无臂回归和 ROS 构建验证。
6. 把旧的调查型交接改为当前状态文档，并在 `CLAUDE.md` 中清理旧 `z=0.18`、±52°、旧相机外参有效等说法。
7. 当前正在把 SO-101 专用交接机制泛化为每次 agent 会话都使用的 `docs/handoff.md`，并增加 `AGENTS.md -> CLAUDE.md` 统一入口。

## 已提交的工作

### `fdd8c1f fix(description): restore lidar geometry and SO-101 deck`

- `laser_joint xyz=-0.012242 0 0.092501`，雷达直接贴装，扫描面离地 143.001 mm。
- 甲板使用后排四孔：底盘 `x=-61 mm`，`y=-24/-8/+8/+24 mm`，相邻距 16 mm。
- STL 名义孔径约 Ø4.3；打印孔 Ø4.5，使用 M4 螺丝、垫圈和螺母。
- 甲板范围 `x=-195..-50 mm`、`y=±48 mm`、厚 8 mm；与雷达平面间隙 10.95 mm。
- 全 ±110° shoulder-pan 扫转最差雷达头间隙 36.2 mm。
- 扫描掩膜由旧 ±52° 改为真实几何推导的 ±24°。
- 标定前 xacro、参数化 CAD、派生模型、掩膜脚本与文档均已入库。

### `503e037 docs: replace SO-101 handoff with current-state guide`

- 删除旧 layout v2 调查日志式交接。
- 增加当前状态交接并同步 `CLAUDE.md`。
- 该提交已经推到 `origin/feat/voice-vla-extensions`。

## 当前未提交的工作

- 把 `docs/so101_handoff.md` 重命名并重写为本文件 `docs/handoff.md`。
- 把 `CLAUDE.md` 的 handoff 规则从 SO-101 专用改成全项目、每次会话通用。
- 创建根目录 `AGENTS.md` 软链接到 `CLAUDE.md`，让不同 agent 读取同一规则。
- 与上述工作无关的 `loc_check.json` 仍是用户修改，必须排除。

## 已验证结果

- CAD：甲板单零件，包围盒 145×96×23 mm；四孔 Ø4.5；三段相邻孔距均为 16.0 mm。
- 视觉检查：甲板板面、通孔、绑带槽、双纵肋和后横 web 完整；整机 GLB 装配树正确。
- 间隙脚本：最差雷达头间隙 36.2 mm。
- 掩膜脚本：几何半宽 18.11°，加 5° 余量后建议 ±24°；边界 `[2.722714, 3.560472]` rad。
- `check_urdf`：正式 URDF 与 viewer URDF 均通过。
- `with_so101:=false`：无 SO-101 链接混入。
- `colcon build --packages-select mentorpi_description`：通过。
- 提交前 `git diff --check`：通过。

## 还遗留什么

### 仓库收尾

1. 检查本次通用 handoff、`CLAUDE.md` 和 `AGENTS.md` 软链接。
2. 用户确认后提交；提交时只包含这些交接规则变更，继续排除 `loc_check.json`。

### 实物与部署

1. **甲板干装**：确认四孔下方能放垫圈/螺母，按底盘板厚 + 8 mm 甲板选择 M4 螺丝长度，并检查线束干涉。
2. **相机**：先确定永久支架，再重新做 AprilTag 外参标定；当前 `camera_joint` 是已拆云台的失效历史值。
3. **2S 电池托架**：实测中层甲板厚度和电池长/深/高，更新 `battery_tray_2s.py` 的 `HOOK_THROAT` 与 `PACK_*`。
4. **Pi 部署**：安装 `ros-jazzy-laser-filters`，构建相关包，以 `with_so101:=true` 启动并同步 systemd 参数。
5. **装车验证**：确认 `/scan_raw -> scan_mask -> /scan`，实测 ±24° 能去除自体回波且可接受后向盲区。
6. **地图**：相机重标、机械臂装车和扫描高度变化后重扫 2D/3D 地图；旧 3D 数据视为失效。
7. **后续**：2S 电气台架、Feetech 舵机 ID/零位/限位、LeRobot 接入、折叠运输姿态；需要规划时再做 MoveIt SRDF。

## 下个 agent 如何继续

```bash
git status --short --branch
git log -5 --oneline --decorate
```

然后：

1. 先区分用户修改与 agent 修改；不要碰 `loc_check.json`。
2. 阅读本文件，再打开当前任务直接涉及的源码和专项文档。
3. SO-101 机械细节以 `mechanical/README.md` 顶部 layout v3、`mechanical/urdf/design-ledger.md` 和参数化源为准。
4. 若继续 CAD/URDF：先改源、再生派生物、运行确定性检查与快照、验证 URDF/无臂回归、最后更新本 handoff。

## 关键文件

- 会话交接：`docs/handoff.md`
- Agent 规则：`CLAUDE.md`；`AGENTS.md` 应为它的软链接
- SO-101 装配：`src/mentorpi_description/urdf/mecanum.xacro`
- 甲板 CAD：`mechanical/printable/so101_deck_plate.py`
- 间隙：`mechanical/measurements/check_so101_clearances.py`
- 扫描掩膜：`mechanical/measurements/compute_scan_mask_so101.py`、`src/mentorpi_bringup/config/scan_mask_so101.yaml`
- 相机重标：`docs/calibration.md`
