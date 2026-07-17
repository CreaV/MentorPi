# MentorPi + LeRobot SO-101 mounting kit — layout v2

This directory is the mechanical source of truth for the SO-101 on-vehicle
integration. STEP is primary; STL is a derived print export. Every printable
part keeps a regenerable parametric Python source — after any caliper
correction, edit the constants and re-run the generators.

## Layout v2 (2026-07-18) — decisions

Layout v2 replaces the v1 (Codex) rear-facing-arm + lidar-riser concept:

- **Arm faces forward** (mount yaw 0 at chassis `x=-0.155`): the Gemini 2L is
  the natural VLA context camera; the gripper enters frame when reaching.
  Mount x was chosen by real-mesh pan-sweep clearance analysis
  (`measurements/check_so101_clearances.py`): worst-case over the full ±110°
  shoulder_pan sweep is 23.1 mm to the lidar head and 11.4 mm to the assumed
  tower posts.
- **Lidar stays at z=0.18** on its stock tower — existing 2D/3D map assets
  remain valid and low-obstacle detection is not degraded. The stowed arm's
  rear scan sector (±52°, covers the zero-pose ±47° crossing) is removed by a
  `laser_filters` mask in `base.launch.py` (`with_so101:=true`).
- **Arm power is a dedicated 2S LiPo** (native 7.4 V for STS3215) instead of
  the v1 USB-PD 20 V → 7.5 V chain: no PD current ceiling (a multi-servo
  stall would hard-trip a 65 W PD port and drop the whole arm), fewer parts,
  and the pack doubles as the front counterweight against the rear arm.
  Branch: `2S pack -> XT60 -> 7.5 A fuse -> latching switch -> Feetech board`.
  USB from the Pi (via hub) is data + signal ground only.

Deprecated v1 parts kept for provenance: `arm_adapter_plate` (superseded by
`so101_deck_plate`), `lidar_riser` (not used — lidar stays put),
`anker_prime_front_tray` (superseded by `battery_tray_2s`),
`xiaomi_hub_carrier` (placeholder; revisit when the hub model is known).

## Printable outputs (v2)

| Part | Purpose | Print guidance |
|---|---|---|
| so101_deck_plate | Rear cantilever deck: shim under the (assumed) lidar tower anchors, SO-101 clamp zone aft | PETG-CF preferred, 6 walls, 50% gyroid, print flat (ribs up) |
| battery_tray_2s | Front-hung 2S LiPo tray / counterweight | PETG, 5 walls, 35% gyroid; two straps mandatory |

Chassis-frame placement (URDF `with_so101:=true` is the reference):

- Deck spans chassis x −195..−15 mm, y ±48, top at z=+65.5 (8 mm plate on the
  57.5 mm top plate). Anchors = the four Codex-inferred Ø4.3 features at
  x=−50.9/−30.9, y=±24.35 (M4 through + top counterbore); assumed shim mode —
  the lidar tower standoffs re-bolt through the same holes on top of the
  plate. Cantilever (92 mm past the chassis rear edge) is stiffened by twin
  15 mm underside ribs + rear web; ribs stop 2 mm shy of the rear face.
- Tray back face rests on the chassis front face (x=+109), L-hooks over the
  top edge (throat 6 mm — caliper the wall thickness), pack top at z=+57.5,
  ~70 mm below and ~60° outside the camera's optical axis.

## Interface audit before printing (P0)

The four chassis anchors are inferred from the repository STL, **not
measured**. Before printing the deck:

1. Identify each candidate feature: through hole / threaded / blind. Measure
   centers, diameters, usable depth, top-plate thickness.
2. Establish whether the lidar tower standoffs actually bolt through these
   holes (shim mode) or elsewhere (then add pass-through holes via
   `CHASSIS_HOLES`-style parameters and re-run).
3. Caliper the front-wall thickness for the tray hook throat (`HOOK_THROAT`)
   and confirm the purchased 2S pack dimensions (`PACK_*`).

## Assembly and commissioning

1. Print a hole gauge (or the deck at 20% infill) first; never force screws.
2. Unbolt the lidar tower standoffs, seat the deck, re-bolt through the deck
   counterbores (longer M4 screws as needed). Strap the SO-101 base in
   addition to its M4 grid.
3. Hang the front tray, dry-fit the hook throat, clamp the M4 screws, insert
   the pack with two straps; check wheel clearance and ground clearance.
4. Wire the 2S branch with the switch off; verify polarity and fuse, connect
   one servo, then the full chain.
5. Re-run `check_so101_clearances.py` with corrected `POSTS`, verify the real
   camera image bottom edge (tray must not appear), verify /scan self-mask
   (`config/scan_mask_so101.yaml` angles) with the arm stowed, then repeat
   drivetrain calibration (`scripts/acceptance_square.py`) under load.
6. Use a folded transport pose; manipulation happens parked (motion-primitive
   safety model). Cap joint velocity/torque for first tests.

This is a geometrically validated first-pass kit, not a structural or
electrical safety certification.
