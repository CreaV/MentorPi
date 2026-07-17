# MentorPi + LeRobot SO-101 mounting kit

This directory is the mechanical source of truth for the first printable
MentorPi/SO-101 integration. STEP is primary; STL is a derived print export.

## Layout

- The SO-101 deck is centered at base_link x=-40.9 mm, y=0, over four candidate
  mounting features identified in the repository base STL at approximately
  x=(-51.0,-30.9) mm, y=+/-24.4 mm. The x=-50.9 mm pair appears only about
  1.5 mm deep in that mesh, while the x=-30.9 mm pair appears through-going.
  Disassemble and measure the real chassis before treating any feature as a
  threaded or through hole. The 8 mm deck uses counterbored M4-clearance holes
  and the standard SO-101 table clamps; two 20 mm straps are a mandatory
  secondary restraint.
- The arm is yawed 180 degrees so the shoulder axis sits toward the rear. This
  keeps the fixed base away from the calibrated front camera.
- The Anker pack is a low front counterweight, across the vehicle and below the
  camera body. The default is Anker Prime A1335 (12,000 mAh/130 W,
  134.4 x 55 x 34.53 mm, 360 g). If the label is not A1335, edit the three
  PACK_* constants before printing.
- The MS200 scan plane moves from 180 to 350 mm above base_link using the
  170 mm riser. The camera transform is not changed.

## Printable outputs

| Part | Purpose | Print guidance |
|---|---|---|
| arm_adapter_plate | Chassis holes to SO-101 clamp deck | PETG/PETG-CF, 6 walls, 50% gyroid, flat |
| anker_prime_front_tray | Low front counterweight cradle | PETG, 5 walls, 35% gyroid, two 20 mm straps |
| lidar_riser | Raise MS200 scan plane by 170 mm | PETG-CF preferred, 6 walls, 45% gyroid |
| xiaomi_hub_carrier | Model-agnostic hub strap tray | PETG, 4 walls, 25% gyroid |

Use M4 washers and nyloc nuts on the arm deck. Dry-fit the front tray hooks
before loading the battery; the STL establishes the chassis envelope, but the
front lip thickness still needs a caliper check on the physical robot. Never
rely on the hooks without both straps.

## Power selection and wiring

The two-USB-C Anker Prime description matches A1335. Each port advertises up to
20 V / 3.25 A, but only 5 V / 3 A; therefore it is not a full-power Pi 5
5 V / 5 A source.

Recommended arm branch:

Anker USB-C1 -> HUSB238 PD trigger at 20 V -> 7.5 V buck -> 7.5 A fuse ->
latching arm switch -> Feetech controller power input

- Preferred buck: Pololu D42V110F7-class 7.5 V high-current regulator, or an
  equivalent synchronous module rated for 20 V input and at least 8 A output.
  A D36V50F7-class 5 A regulator is acceptable only with conservative torque
  limits and may brown out on simultaneous stalls.
- Use 18 AWG silicone wire on the 7.5 V branch, keep the fuse near the buck
  output, and set the regulator before connecting any servo.
- The Feetech USB adapter is data only; its motor bus still requires the power
  branch. USB establishes signal ground. Do not feed the arm from Pi USB,
  RRCLite's private bus-servo port, or the Raspberry Pi 5 V rail.
- Keep Pi 5 on the existing RRCLite 5 V supply while mobile (or the official
  27 W supply on the bench). A1335's 5 V/3 A PDO would restrict Pi USB power.
- USB-C2 may power a hub only if the exact Xiaomi hub has a documented PD-in
  port with upstream back-feed isolation. Otherwise leave the hub bus-powered
  for data only.

USB topology:

Pi blue USB3 -> Gemini 2L directly

Pi second USB -> Xiaomi hub -> Feetech adapter + low-bandwidth receivers

Keep the camera direct because this repository already records severe dual-stream
bandwidth loss through the tested hub. A genuinely powered USB3 hub remains the
fix for high-current camera/lidar peripherals.

The A1335 does not support pass-through operation: stop the arm and disconnect
its switched branch before charging the power bank.

## Assembly and commissioning

1. Inspect the real chassis and identify whether each candidate feature is a
   shallow recess, threaded hole, or through hole. Measure its center and usable
   depth before selecting fasteners.
2. Print the deck or a hole gauge first. Proceed with bolting only after the
   real interface is confirmed; then clamp and strap the official SO-101 base.
3. Install the unloaded front tray, check wheel sweep and at least 40 mm ground
   clearance, then add the power bank and two straps.
4. Fit the lidar riser and confirm the physical scanner center is 170 mm higher.
5. Wire power with the arm switch off. Verify 7.5 V polarity and current limit,
   then connect one servo before the full daisy chain.
6. Use a folded transport pose, lock /cmd_vel during manipulation, cap joint
   velocity/torque for the first tests, and repeat drivetrain calibration after
   the payload is installed.

This is a geometrically validated first-pass kit, not a structural or electrical
safety certification. Re-run the generators after any caliper correction.
