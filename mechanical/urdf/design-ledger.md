# MentorPi + SO-101 URDF design ledger

## Layout v3 (2026-07-19, current)

- `base_link -> laser_frame` is restored from the recovered pre-calibration
  xacro: `xyz=-0.012242 0 0.092501`, `rpy=0 0 0`. The base STL top and lidar
  mesh bottom both resolve to 108.001 mm above ground; the scan plane is
  143.001 mm above ground. The lidar is direct-mounted, with no tower links.
- `base_link -> so101_base_link` remains fixed at
  `xyz=-0.155 0 0.0655`, `rpy=0 0 0`. Full ±110° shoulder-pan clearance to
  the real lidar-head mesh is 36.2 mm worst-case.
- `so101_deck_plate` spans chassis x -0.195..-0.050, y ±0.048, top z=0.0655.
  Its four chassis clearances are Ø4.5 at x=-0.061 and
  y=-0.024/-0.008/+0.008/+0.024, matching the accepted STL measurements.
  These holes are unthreaded, use M4 screws + nuts, and do not mount lidar.
  Plate-to-lidar plan clearance is 10.95 mm; rear cantilever remains 92 mm.
- The arm-enabled rear scan mask is ±24° (0.418879 rad), derived by intersecting
  the actual chassis and SO-101 mesh triangles with the corrected scan plane.
- The old `camera_joint` transform belonged to the previous camera mount and is stale.
  Its mesh-intrinsic cosmetic offset remains valid, but the joint must be
  recalibrated only after the final camera bracket is fixed.
- Printed-mount inertials remain provisional estimates pending measured mass.

The layout v2 and v1 sections below are retained only as superseded history.

---


## Layout v2 (2026-07-18, supersedes the v1 entries below where they conflict)

- Arm faces forward: `base_link -> so101_base_link` fixed at
  `xyz=-0.155 0 0.0655`, `rpy=0 0 0`. Mount x from real-mesh pan-sweep
  analysis (`mechanical/measurements/check_so101_clearances.py`): worst
  ±110° clearance 23.1 mm to the lidar head, 11.4 mm to the assumed posts.
- `laser_frame` stays at z=0.180 in ALL configs (no riser). The stowed arm's
  rear scan sector is masked by laser_filters (`with_so101:=true` in
  base.launch.py); zero-pose crossing sector is ±47° about the rear axis,
  mask is ±52°.
- Deck plate `so101_deck_plate` spans chassis x -0.195..-0.015, y ±0.048,
  top z=0.0655; assumed shim mode under the lidar tower anchors (the four
  Codex-inferred holes — PHYSICAL AUDIT PENDING). Rear cantilever 92 mm.
- Arm power: dedicated 2S LiPo in `battery_tray_2s` hung on the chassis
  front face (back face at x=0.109, pack top z=0.0575), doubling as front
  counterweight. Pack envelope provisional (105x33x24, 2S2200).
- Camera visual note: `camera_link` is the CALIBRATED optical center; the
  body mesh is drawn with a cosmetic offset (-0.0137, -0.0305, +0.013) so it
  renders at the physical body position. Never "fix" the calibrated joint to
  make the mesh look centered.
- riser / anker tray / adapter plate links removed from the URDF; sources
  remain in mechanical/printable/ as deprecated provenance.

## Layout v1 (Codex, 2026-07-17, superseded)

- Consumer: RViz/Foxglove and `robot_state_publisher`; simulation inertials are
  inherited from the official SO-101 model and remain provisional for the
  custom printed mounts.
- Units: URDF SI (m, kg, rad); all copied SO-101 meshes are already meter-scaled.
- Base convention: existing MentorPi REP-103 `base_link` is unchanged.
- Mount: `base_link -> so101_base_link`, fixed at `xyz=-0.0409 0 0.0655`,
  `rpy=0 0 pi`. The XY location is the center of four candidate mounting
  features inferred from the repository base STL, not a physical measurement.
  The x=-50.9 mm pair appears shallow in that mesh and must be checked on the
  real chassis. Z is the reference-mesh top surface (57.5 mm) plus the 8 mm
  adapter.
- Lidar with arm enabled: `laser_frame` moves from z=0.180 m to z=0.350 m,
  matching the 170 mm riser. Camera and calibrated camera transform are unchanged.
- SO-101 moving joints, axes, limits, masses, COMs, inertias and visual origins
  are copied from `TheRobotStudio/SO-ARM100` commit `fda892c`, new-calibration URDF.
- Positive motion and names follow the upstream model: `shoulder_pan`,
  `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`, all
  prefixed `so101_` to avoid collisions.
- `so101_gripper_frame_link` is intentionally frame-only; the upstream dummy
  near-zero inertial was removed.
- Assumption: adapter mounting surface is coincident with the SO-101 base frame
  z=0.  Confirm after the first dry assembly and adjust only the named mount
  constants if a measured offset appears.
