# 3D 建图 → 高斯泼溅 (3DGS) → 交互式浏览管线

目标：把 rtabmap 3D 建图结果离线训练成高斯泼溅模型（高性能服务器上做），
之后每次重启 → `loc_3d` 重定位 → 在查看端 (PC/手机) 的 Rerun 里实时看到
机器人在同一个 3D 场景中的准确位置 + 相机视场角投影。渲染全部发生在查看
端，树莓派只出 TF 和压缩视频流。

```
┌─ 机器人 (Pi 5) ──────────────┐   ┌─ 训练服务器 (GPU) ─────────────┐
│ slam_3d 建图                  │   │ ns-train splatfacto            │
│   → ~/rtabmap_maps/xxx.db ───┼──→│   (map 坐标系, 禁止自动重定向)  │
│                               │   │   → splat.ply (map 坐标系)     │
│ loc_3d 重定位                 │   └───────────────┬────────────────┘
│   → TF: map→odom→base_link   │                   │
│   → rosbridge :9090 ─────────┼──→ ┌─ 查看端 (PC / 手机浏览器) ─────┐
└───────────────────────────────┘   │ scripts/live_rerun.py          │
                                    │   splat.ply + SLAM 点云 (静态)  │
                                    │   + 实时机器人位姿 + 相机视锥   │
                                    └────────────────────────────────┘
```

**坐标系是整条管线的命门**：`export_gs_dataset.py` 导出的相机位姿保持在
ROS `map` 坐标系（米制、z-up），训练时必须禁用 nerfstudio 的自动重定向/
缩放，这样训练出的 splat 天然和 SLAM 地图对齐 —— 重定位后 rtabmap 发布的
`map → odom` TF 直接把机器人"放进" splat 场景里，无需任何后期配准。

## 1. 建图（机器人上）

用 `slam_3d` 模式正常建图（supervisor 或 CLI 均可）。给 3DGS 采数据的建议：

- **慢速多角度**：3DGS 质量取决于视角覆盖，同一区域从不同方向各扫一遍
- 光照稳定、避免大面积白墙（loop closure 和 GS 都依赖纹理）
- 建完确认 rtabmap 已回环优化（Foxglove 里看 `/rtabmap/mapGraph`）
- 数据库在 `~/rtabmap_maps/rtabmap.db`（或 SetMode 指定的 database_path）

## 2. 导出训练数据集

在任何装有 rtabmap 工具的机器上（Pi 或 PC 都行；PC 更快，先把 .db 拷过来）：

```bash
sudo apt install ros-jazzy-rtabmap        # 只要 rtabmap-export 工具，不需要 ROS 运行时
python3 scripts/export_gs_dataset.py ~/rtabmap_maps/rtabmap.db --output-dir ~/gs_dataset
```

产物（nerfstudio 格式）：

| 文件 | 内容 |
|------|------|
| `transforms.json` | 回环优化后的相机位姿 (OpenGL c2w, **map 坐标系**) + 每帧内参 |
| `images/<id>.jpg` | RGB 关键帧（装了 OpenCV 会自动去畸变） |
| `depth/<id>.png` | 16UC1 深度 (mm)，与 RGB 对齐 |
| `sparse_pc.ply` | RGB-D 拼接点云 → splatfacto 的初始化种子点 |

把 `~/gs_dataset` 整个 scp/rsync 到训练服务器。

## 3. 服务器训练 (nerfstudio splatfacto)

```bash
# 一次性安装 (conda + CUDA 环境)
pip install nerfstudio

# 训练 —— 三个 flag 缺一不可，它们保证输出停留在 map 坐标系:
ns-train splatfacto --data ~/gs_dataset \
    nerfstudio-data \
    --orientation-method none \
    --center-method none \
    --auto-scale-poses False

# 导出 splat (输出 .ply, 3DGS 格式)
ns-export gaussian-splat \
    --load-config outputs/gs_dataset/splatfacto/<timestamp>/config.yml \
    --output-dir exports/splat/
```

拿到 `exports/splat/splat.ply`，拷回查看端。

备选训练器（不想装 nerfstudio 时）：
- [OpenSplat](https://github.com/pierotofy/OpenSplat)：C++，直接吃 nerfstudio
  格式目录，`opensplat ~/gs_dataset -n 30000`，同样不做坐标变换
- gsplat 官方 example 脚本（`--data_factor 1`，注意它默认 COLMAP 格式，
  需要 nerfstudio dataparser）

**对齐自检**：训练完把 `splat.ply` 和 `sparse_pc.ply` 同时丢进
`live_rerun.py`（`--splat` + `--cloud`），两者应该完全重合。不重合 =
训练时没关自动重定向。

## 4. 交互式浏览 + 实时位姿（查看端）

依赖（查看端 PC，无需 ROS）：

```bash
pip install rerun-sdk roslibpy numpy plyfile
```

机器人保持 `remote.launch.py` 运行（rosbridge :9090 已就绪），重启后先切
`loc_3d` 模式重定位（手机 SPA 上点 "Loc 3D" 选 .db，或桌面调用
`/mode/set`）：

```bash
# PC 本地 Rerun 窗口
python3 scripts/live_rerun.py --robot <robot-ip> \
    --splat splat.ply --cloud rtabmap_maps/rtabmap_cloud.ply

# 手机/平板浏览: 加 --serve, 用打印出的 URL 在手机浏览器打开
python3 scripts/live_rerun.py --robot <robot-ip> --splat splat.ply --serve
```

看到的内容（全部在 map 坐标系）：
- `map/splat`：高斯泼溅模型（以彩色点云渲染，见下方局限）
- `map/slam_cloud`：rtabmap 原始点云（可在 Rerun 里按需开关图层）
- `map/odom/base_link/...`：实时 TF 链 = 机器人当前位姿
- 相机 optical frame 上的 **Pinhole 视锥**（真实 FOV，来自 camera_info）
  + 实时 JPEG 视频贴在视锥上
- `map/trajectory`：行驶轨迹线

注意：机器人处于 `idle`/2D 模式时没有 `map→odom`，机器人会显示在 map 原
点上的 odom 系里；切 `loc_3d` 且 rtabmap 完成重定位后位置才是真的。

### 高斯泼溅渲染质量的局限

Rerun 把 3DGS ply 当**彩色点云**渲染（取 SH 直流分量做颜色、按 opacity
过滤），交互浏览、验证对齐、看实时位姿足够；但没有各向异性 splat 的
"实心感"。要全质量渲染：

- [SuperSplat](https://playcanvas.com/supersplat/editor)（浏览器，直接拖入
  splat.ply，手机也能跑）
- 若 Rerun 后续版本原生支持 3DGS 渲染，`live_rerun.py` 的点云 fallback 可
  直接替换为 asset 日志，实体路径不变

## 5. 离线调试回放

录制的 mcap bag（`scripts/record_3d_bag.sh`）依旧用：

```bash
python3 scripts/bag_to_rerun.py ~/rosbags/3d_slam/<run>   # 回放整个 bag 到 Rerun
```

## 故障排查

| 症状 | 原因 / 处理 |
|------|------------|
| splat 和 SLAM 点云错位/翻转 | 训练时没加 `--orientation-method none --center-method none --auto-scale-poses False` |
| splat 尺度不对 | 同上（auto-scale 被启用了） |
| live_rerun 里机器人不动 | 机器人不在 loc_3d/slam_3d 模式，或 rtabmap 还没重定位成功（移动一下、经过有纹理区域） |
| 训练图像糊 | 建图时开快了；重扫，或用 `rtabmap-export --texture_blur 50` 类似思路先筛帧 |
| `addWordRef() Not found word` | db 累积错位，备份重建（见 CLAUDE.md） |
