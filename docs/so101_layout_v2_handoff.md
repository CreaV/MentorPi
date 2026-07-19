> 2026-07-19 layout v3 closure
>
> This handoff records the investigation path. The current conclusions below
> supersede the unresolved layout v2 branches in §3 and §6:
>
> - §3.1 resolved from the recovered pre-calibration xacro plus STL geometry:
>   `laser_joint xyz=-0.012242 0 0.092501`; scan plane 143.001 mm above ground;
>   lidar is directly mounted and all fake tower visuals are removed.
> - §3.2 remains open: the old camera joint belonged to the removed gimbal and
>   must be recalibrated after the final camera bracket is fixed.
> - §3.4 resolves to branch B. The rear holes do not mount lidar; the deck ends
>   at chassis x=-50 mm, leaving 10.95 mm plan clearance to the lidar mesh.
> - The accepted STL hole row is x=-61 mm, y=-24/-8/+8/+24 mm, nominal Ø4.3,
>   16 mm pitch. The real holes are unthreaded through holes; the deck uses
>   Ø4.5 clearance with M4 screws + nuts. The separate front two are unused.
> - §3.5 resolved: the real-geometry arm mask is ±24°, and the corrected full
>   shoulder-pan sweep has 36.2 mm worst clearance to the lidar head.
> - §3.6 items 1 and 2 are closed without further measurement. Items 3–6
>   remain physical integration inputs; only item 3 blocks camera recalibration.
>
> Do not execute the old §6 steps 1–2; the CAD, URDF and mask are already
> regenerated from the conclusions above.
>
---

# SO-101 集成布局 v2 —— 交接文档

- 日期：2026-07-18
- 分支：`feat/voice-vla-extensions`
- 提交范围：`0ce9b9d` .. `9353eac`（本次会话 9 个提交，全部已 commit，未 push）
- 前置：Codex 初版方案 + 评审见 `docs/mentorpi_so101_integration_report.md`
- 设计决策 + 几何分析：`mechanical/README.md`、`mechanical/urdf/design-ledger.md`

---

## 1. 这次做了什么

在 Codex 初版 SO-101 车载集成方案的基础上，按用户拍板的三个反向决策重做了
布局 v2（软件链路 P1 + CAD 重设计 P2），并在可视化审阅中发现并修正了 URDF 里
的两处装饰件建模错误，最终暴露出一个**存在了约三周的雷达 TF 高度 bug**。

### 1.1 三个核心决策（用户拍板）

| # | Codex 初版 | v2 决策 | 理由 |
|---|-----------|---------|------|
| 臂朝向 | 朝后（yaw 180°，避开相机） | **朝前**（yaw 0） | Gemini 2L 天然成为 VLA context 相机；前伸时夹爪入画 |
| 雷达 | 加 170mm 增高座 | **不动 + 角度掩膜** | 保住已建地图资产；不新增机械件；避障不降级 |
| 臂供电 | USB-PD 20V→7.5V 降压链 | **独立 2S 锂电** | STS3215 原生 7.4V；无 PD 65W 硬切断风险；兼前配重 |

### 1.2 提交清单

| 提交 | 内容 |
|------|------|
| `0ce9b9d` | Codex 初版原样入库存痕（provenance），排除用户的 loc_check.json |
| `a17bdaf` | URDF v2：臂朝前 `x=-0.155`、雷达留 z=0.18、so101 include 移入 if 块、相机机身视觉偏移修正、雷达塔架示意柱 |
| `cc889d2` | launch 链路：`with_so101` 从 remote→base→xacro 透传；laser_filters 后向掩膜（`/scan_raw`→`/scan`）；config/scan_mask_so101.yaml |
| `b239f01` | CAD：`so101_deck_plate`（悬伸甲板）+ `battery_tray_2s`（2S 托架）；间隙分析脚本 `check_so101_clearances.py`；README/ledger/CLAUDE.md 重写 |
| `7941728` | （已作废）CAD viewer 软链尝试 |
| `c906548` | viewer 专用 URDF 变体（package:// → 相对路径） |
| `140cb62` | 预览 URDF 剥离 collision 加速；相机云台底座示意件 |
| `e8ca78b` | `bake_urdf_glb.py`：URDF 烘焙成静态 GLB，绕过所有 URDF 加载器 |
| `9353eac` | 修两处真悬空：云台舵机塔 + 电池挂线改到中层甲板前缘 |

### 1.3 P1 软件链路（可直接部署，不依赖机械件）

- **`with_so101` 全线打通**：`remote.launch.py` → `base.launch.py` → xacro。
  默认 `false` 时 TF/扫描链路与无臂**完全一致**（已验证 false 配置渲染与
  改动前逐字节相同）。systemd 启用 = ExecStart 加 `with_so101:=true`。
- **雷达自体掩膜**：装臂后 oradar 发 `/scan_raw`，`laser_filters`
  `scan_to_scan_filter_chain` 掩膜后向扇区后再发 `/scan`，下游
  （slam_toolbox / rtabmap / obstacle guard）无感知。角度见
  `config/scan_mask_so101.yaml`（三段区间兼容 0..2π 与 -π..π 表示）。
  **部署依赖**：`sudo apt install ros-jazzy-laser-filters`（Pi 上尚未安装）。
- **xacro 惰性求值**：`so101.xacro` 的 include 放在 `<xacro:if with_so101>`
  块内，未启用时不依赖该文件存在。

### 1.4 P2 CAD（参数化，STEP 主 + STL 派生）

- `mechanical/printable/so101_deck_plate.py`：180×96×8mm 后悬伸甲板，
  shim 模式共用雷达塔架 4 锚点，92mm 悬伸段双肋 + 后横web 加强。
- `mechanical/printable/battery_tray_2s.py`：2S 前挂托架兼前配重，L 型
  挂钩（喉深 6mm 参数化）。
- 臂 mount `x=-0.155` 由真网格 shoulder_pan ±110° 全扫掠选定（脚本
  `mechanical/measurements/check_so101_clearances.py`）。**注意：该分析假设
  雷达在塔架上，现已证伪，见 §3。**

---

## 2. 关键教训：视觉件 vs 标定值

会话中反复出现"相机/雷达位置看起来离谱"的观感。**结论：标定值从未错，也
从未为任何视觉效果改动过。** 但过程中我犯了错，也发现了真 bug，如实记录：

### 2.1 正确的部分

- **相机 `camera_joint`**（`xyz=0.1114 0.0305 0.0950 rpy=-0.0164 -0.1302
  0.0147`）：2026-07-17 AprilTag 手眼标定，8 位姿 RMS 9.7mm，且经**地面点云
  反投影验证**（平整度 1.3cm、倾斜 0.01°）。这两个是独立物理实验，是标定
  正确性的证据——**渲染图对标定零证明力**。
- `cam_Link.STL` 机身居中建模 vs `camera_link`=光学中心（镜头偏 y+3cm、
  上仰）→ 加了 cosmetic visual 偏移让机身画在物理位置。**勿动
  camera_joint 去"修"外观。**

### 2.2 我犯的错（装饰件建模错误）

用户从烘焙 GLB 里发现两处**真悬空**（不是渲染 bug）：

1. **电池托架挂在空气上**：托架挂钩瞄准"垂直前脸"，但网格切片证明底盘
   鼻部是**台阶状**（顶板 z=57.5mm 只到 x=+17，中层甲板 z=25.3mm 到 x=+80，
   最前只有低保险杠，**无垂直前脸**）。已改挂中层甲板前缘（`9353eac`）。
2. **相机云台**：先只画了 2cm 占位盒，后补成 52mm 全高舵机塔。

**流程反思**：前两轮我"先假设后验证"（软链、Explode 开关、"数据一定对"），
应该一开始就自己离线渲染取证。离线正交渲染脚本
`mechanical/urdf/bake_urdf_glb.py` + `offline_render_check.png` 是最终定论
工具——任何 URDF 改动后先自己渲染，别依赖第三方查看器。

---

## 3. 遗留问题（阻塞项，需用户实测）

### 3.1 ⚠️ 最高优先级：雷达 TF 高度可能从第一天就错了

**用户 2026-07-18 实测：雷达底部实际高度 ≈11.2cm，紧贴车身，没有立柱。**

- xacro 里 `laser_joint z=0.18`（折算雷达悬在顶板上方 ~9cm）与实测矛盾。
  11.2cm ≈ 顶板高(10.8cm) + 薄垫片 = **直接贴装**。
- 为什么三周没被发现：**2D SLAM / EKF / 避障守卫全是平面计算，laser 的 z
  从不进任何公式**；唯一消费 z 的是 rtabmap 的 3D 显示。xacro 注释"z 已实测"
  是错的（疑似沿用厂商别车型样板值）。
- **待办**：量到 MS200 扫描面精确高度后改 `laser_joint z`（预计 ~0.085–0.09）。

### 3.2 ⚠️ 相机云台已被用户物理拆除 → 外参作废

- 用户拆掉了 2-DOF 云台舵机塔，相机现在**紧贴车身**。
- 07-17 的 `camera_joint` 外参对应的是"相机在云台上"的位置，**即刻作废**。
- **待办**：新固定方式定型后，AprilTag 重标
  （`scripts/calibrate_camera_extrinsic.py --update-xacro`）。装回去就白标，
  务必等最终固定方式确定。

### 3.3 地图资产连锁作废

| 资产 | 状态 |
|------|------|
| `room_20260717.db` | 相机移位 + 扫描高度变 → 作废，需重标后重扫 |
| 2D 旧图（.posegraph） | 部分存活（2D 匹配不吃 z；但家具在新扫描高度轮廓不同），建议重扫 |

### 3.4 甲板设计依赖未验证的孔位假设

`so101_deck_plate` 整个是按"雷达装在塔架上、4 孔是塔架锚点"设计的。雷达实际
贴装 → **甲板前缘会和雷达占地相撞**。两个分支取决于 P0 实测：

- **分支 A（很可能）**：4 孔就是雷达自己的固定孔 → 甲板做成真垫片垫在
  雷达底下（只抬 8mm），前缘延伸覆盖雷达区。
- **分支 B**：孔另有用途 → 甲板止步 x≈-50mm，完全避开雷达。

### 3.5 掩膜扇区需重算

扫描面从 23cm 降到 ~13.5cm 后，臂基座穿越姿态变化 → `scan_mask_so101.yaml`
的 ±52° 需重算。附带：低障碍盲区从 23cm 降到 ~13.5cm（收益）；之前 23mm 的
臂-雷达头紧张裕量消失（雷达头位置也变了）。

### 3.6 P0 卡尺实测清单（一次量齐，避免反复拆装）

1. **雷达固定孔**：后部 4 孔（约 x=-51/-31、y=±24）是不是雷达固定孔？孔距、
   孔径、有效深度、是否螺纹。
2. **雷达扫描面精确高度**（配 MS200 规格推算，能看到光学窗中心更好）。
3. **相机最终安装位 + 离地高度**（云台是否永久拆除？定了才能重标）。
4. **中层甲板板厚**（电池托架钩喉 6mm 是估的）。
5. **顶板厚度**（甲板锚点螺丝长度）。
6. **2S 电池实物尺寸**（改 `PACK_*` 常量）。

---

## 4. 未做 / 后续阶段（未阻塞，未开工）

- **P3 打印**：`cad:gcode` 切片 + `cad:bambu-labs` 上机。**需先确认打印机是否
  Bambu 且同局域网。**
- **P4 电气台架**：2S + XT60 + 保险丝 + 带锁开关 → Feetech 板；空载验电压、
  单舵机、再全链。
- **P5 装配后系统验证**：真实画面查视野下沿、guard/掩膜实测、带载
  `acceptance_square.py` 重标定、质量台账/翻覆裕度。
- **P6 LeRobot 接入**：Feetech 挂小米 Hub、舵机 ID/标定、腕部相机、AIRE
  `robot` skill 扩展臂原语；MoveIt 阶段用 `cad:urdf`/`cad:srdf`。

---

## 5. 工具 / 环境备忘

- **CAD 工具链**：`/tmp/mentorpi-cad-venv/bin/python` + text-to-cad skills
  （`~/.claude/plugins/cache/text-to-cad/cad/0.3.9/skills/`）。`scripts/step`
  的 `--stl` 路径相对 target 目录，不是 cwd。
- **CAD Viewer**（127.0.0.1:4178）：**前端不解析 `package://` URI**（带 scheme
  直接落 dist 静态根 404）。URDF 预览用 `mentorpi_so101.viewer.urdf`
  （`gen_mentorpi_so101_viewer.py` 把 `package://mentorpi_description/` 重写成
  `../`），页面 `file=` 用相对 dir 路径。
- **URDF 加载器 bug**：CAD Viewer 和 VS Code URDF Visualizer（同 three.js
  URDF-loader 谱系）都把 fixed 关节子树散开显示。**烘焙 GLB
  （`bake_urdf_glb.py`）是唯一可靠的整机预览**（Y-up 已转正，静态无关节）。
- **派生物 gitignore**：`.preview.glb`、`.*.step.glb`。
- **未 push**：本次 9 个提交全部本地，用户的 `loc_check.json` 改动一直保留未碰。

---

## 6. 下一步（建议顺序）

1. 用户完成 §3.6 P0 卡尺清单 → 报数字。
2. 我据此改 `laser_joint z`、重定甲板分支（A/B）、重出 CAD、重算掩膜。
3. 用户定相机最终固定方式 → AprilTag 重标外参。
4. 重扫地图（`room_YYYYMMDD.db`）。
5. 确认打印机 → 进 P3。
