# MentorPi + SO-101 URDF design ledger

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
