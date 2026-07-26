# MentorPi → Isaac Sim / Isaac Lab

Import-ready robot descriptions for NVIDIA Isaac Sim and Isaac Lab, generated
from the **calibrated** URDF source of truth.

## Files

| File | What |
|------|------|
| `mentorpi.isaac.urdf` | **Base only** — mecanum chassis + camera + lidar + IMU (no arm) |
| `mentorpi_so101.isaac.urdf` | **Full mobile manipulator** — base + SO-101 arm |
| `mentorpi_articulation_cfg.py` | Isaac Lab `ArticulationCfg` for both (`MENTORPI_CFG`, `MENTORPI_SO101_CFG`) |
| `export_isaac.sh` | One command that regenerates both URDFs from the xacro |
| `usd/` | Where you save the URDF→USD conversions (git-ignored; you create it) |

Generators live in `mechanical/urdf/gen_mentorpi_isaac.py` and
`gen_mentorpi_so101_isaac.py`.

## Bound to calibration (important)

These URDFs are **not hand-maintained** — they are generated from
`src/mentorpi_description/urdf/mecanum.xacro`, the single source of truth that
holds the calibrated wheelbase / track_width / wheel_diameter, the
`camera_joint` extrinsic, and `laser_joint`. To re-sync after **any**
calibration or geometry change:

```bash
bash isaac/export_isaac.sh
```

Run this in particular after `scripts/calibrate_camera_extrinsic.py
--update-xacro` (camera re-calibration) or any wheel re-calibration. It builds
`mentorpi_description` (so `$(find …)` picks up the current xacro), regenerates
both URDFs, and runs `check_urdf`. Then re-convert to USD (below). Never edit
the generated `*.isaac.urdf` by hand.

Mesh paths are **absolute** into `src/mentorpi_description/meshes/…`, so import
is foolproof on this machine; regeneration adapts the paths if the repo moves.

## URDF → USD

Isaac uses USD. Convert once per URDF (re-convert after re-exporting).

### Option A — Isaac Sim URDF Importer (GUI, stable)

1. Isaac Sim → *Isaac Utils → Workflows → URDF Importer* (or *File → Import*).
2. Input file: `isaac/mentorpi_so101.isaac.urdf` (or `mentorpi.isaac.urdf`).
3. Settings that matter for this robot:
   - **Fix Base Link: OFF** — floating base (mobile robot).
   - **Merge Fixed Joints: ON** — collapses camera/lidar/imu/optical frames
     into `base_link`; keeps the 4 wheels + 6 arm joints articulated.
   - **Joint Drive Type: Velocity** for a nav robot (or Position; Isaac Lab
     overrides drive gains via the ArticulationCfg anyway).
   - Convex decomposition for collisions is fine for the chassis.
4. Output to `isaac/usd/mentorpi_so101.usd`.

### Option B — Isaac Lab CLI (scriptable)

Isaac Lab ships a converter; run it with Isaac's python:

```bash
# from your Isaac Lab checkout (flag names vary slightly by version):
python scripts/tools/convert_urdf.py \
    /media/luo/Game/data/code/MentorPi/isaac/mentorpi_so101.isaac.urdf \
    /media/luo/Game/data/code/MentorPi/isaac/usd/mentorpi_so101.usd \
    --merge-joints --no-fix-base
```

## Use in Isaac Lab

```python
from isaac.mentorpi_articulation_cfg import MENTORPI_SO101_CFG  # or MENTORPI_CFG

# in your scene cfg:
robot = MENTORPI_SO101_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
```

Point `_USD_DIR` in `mentorpi_articulation_cfg.py` at your `usd/` output. Joint
names/limits/init pose already match the URDF; the arm uses effort/velocity=10
(from the URDF) with position drive, wheels use velocity drive.

## ⚠️ Mecanum caveat

Isaac does **not** simulate mecanum rollers. The 4 wheels import as plain
revolute joints, so spinning them all forward gives only longitudinal motion —
no native strafing. For holonomic behaviour you must map `(vx, vy, wz)` to
per-wheel speeds yourself (and/or model the rollers). For pure navigation
research many users drive the base with a velocity/holonomic controller rather
than wheel torques.

## Frames

After merge-fixed-joints, `camera_link`, `laser_frame`, and `imu_link` live on
the `base_link` rigid body at their calibrated offsets:
- `camera_link`: base_link + `xyz 0.1017 0.0137 0.0535` /
  `rpy -0.0171 -0.1196 0.0323` (AprilTag hand-eye calibration 2026-07-24,
  position RMS 9.6 mm; re-export if the camera is ever re-calibrated).
- `laser_frame`: base_link + `xyz -0.012242 0 0.092501`.
Attach Isaac camera/lidar sensors to these prims to match the real robot.
