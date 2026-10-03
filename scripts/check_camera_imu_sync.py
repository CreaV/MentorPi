#!/usr/bin/env python3
"""
Gemini 2 IMU / image timing check — the S0 gate in
docs/vision_mapping_roadmap.md. Run on the Pi (or any host on the ROS graph)
while base is up with camera_imu:=true:

    /usr/bin/python3.12 scripts/check_camera_imu_sync.py [--duration 30]

Keep the robot still for the first run, then repeat while rotating in place.

Reports, per stream (camera IMU, color, depth):
  - rate and inter-message jitter from header stamps, non-monotonic stamps
  - latency = receive time - header stamp. Streams stamped in the same
    (host) clock domain all show small positive latencies; a stream stamped
    with a raw device clock shows a huge or negative offset.
and color↔depth pairing offsets (sizes rgbd_sync approx_sync_max_interval).

This is a necessary, not sufficient, check: equal clock domains and low IMU
jitter make tightly coupled VIO viable, but the true camera-IMU time offset
comes from Kalibr (docs/vision_mapping_roadmap.md S0/S).
"""
import argparse
import bisect
import math
import statistics
import sys
import time

# Thresholds for the verdict (seconds).
SAME_CLOCK_MAX_LATENCY = 0.5   # |latency| beyond this => different clock domain
IMU_JITTER_OK = 0.001          # std of IMU stamp deltas for tight coupling


def stream_stats(stamps, recv):
    """Rate / jitter / latency summary for one stream.

    stamps: header stamps (s), in arrival order. recv: host receive times (s).
    """
    n = len(stamps)
    out = {'count': n}
    if n < 2:
        return out
    dts = [b - a for a, b in zip(stamps, stamps[1:])]
    out['non_monotonic'] = sum(1 for d in dts if d <= 0.0)
    pos = [d for d in dts if d > 0.0]
    span = stamps[-1] - stamps[0]
    out['rate_hz'] = (n - 1) / span if span > 0 else float('nan')
    if pos:
        out['dt_mean'] = statistics.fmean(pos)
        out['dt_std'] = statistics.pstdev(pos)
        out['dt_max'] = max(pos)
    lat = [r - s for s, r in zip(stamps, recv)]
    out['lat_mean'] = statistics.fmean(lat)
    out['lat_min'] = min(lat)
    out['lat_max'] = max(lat)
    return out


def pairing_offsets(a_stamps, b_stamps):
    """For each stamp in a, |offset| to the nearest stamp in b."""
    b = sorted(b_stamps)
    if not b:
        return []
    offs = []
    for t in a_stamps:
        i = bisect.bisect_left(b, t)
        cands = [b[j] for j in (i - 1, i) if 0 <= j < len(b)]
        offs.append(min(abs(t - c) for c in cands))
    return offs


def verdict(imu, color):
    """Return (ok, list of reasons) for the tight-coupling gate."""
    reasons = []
    if imu.get('count', 0) < 2:
        return False, ['no camera IMU data (camera_imu:=true? topic name?)']
    if color.get('count', 0) < 2:
        return False, ['no color camera_info data']
    ok = True
    for name, s in (('imu', imu), ('color', color)):
        if abs(s['lat_mean']) > SAME_CLOCK_MAX_LATENCY:
            ok = False
            reasons.append(
                f'{name} latency {s["lat_mean"]:+.3f}s: stamped in a different '
                'clock domain than the host')
    if imu.get('non_monotonic', 0):
        ok = False
        reasons.append(f'imu has {imu["non_monotonic"]} non-monotonic stamps')
    jitter = imu.get('dt_std', math.inf)
    if jitter > IMU_JITTER_OK:
        ok = False
        reasons.append(f'imu stamp jitter {jitter * 1e3:.2f} ms > '
                       f'{IMU_JITTER_OK * 1e3:.1f} ms')
    if ok:
        reasons.append('same clock domain, monotonic, low IMU jitter — '
                       'proceed to Kalibr for the exact time offset')
    return ok, reasons


def fmt(name, s):
    if s.get('count', 0) < 2:
        return f'{name:6s} count={s.get("count", 0)} (not enough data)'
    return (f'{name:6s} n={s["count"]:6d} rate={s["rate_hz"]:7.1f}Hz '
            f'dt={s.get("dt_mean", float("nan")) * 1e3:6.2f}±'
            f'{s.get("dt_std", float("nan")) * 1e3:5.2f}ms '
            f'(max {s.get("dt_max", float("nan")) * 1e3:6.1f}) '
            f'nonmono={s["non_monotonic"]} '
            f'latency mean={s["lat_mean"] * 1e3:+8.1f}ms '
            f'[{s["lat_min"] * 1e3:+.1f}, {s["lat_max"] * 1e3:+.1f}]')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--imu-topic', default='/camera/gyro_accel/sample')
    ap.add_argument('--color-info-topic', default='/camera/color/camera_info')
    ap.add_argument('--depth-info-topic', default='/camera/depth/camera_info')
    ap.add_argument('--duration', type=float, default=30.0)
    args = ap.parse_args()

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Imu

    rclpy.init()
    node = rclpy.create_node('check_camera_imu_sync')
    data = {k: ([], []) for k in ('imu', 'color', 'depth')}

    def cb(key):
        def _cb(msg):
            st = msg.header.stamp
            data[key][0].append(st.sec + st.nanosec * 1e-9)
            # Host wall clock: same domain the driver stamps into when it
            # uses system/global time.
            data[key][1].append(time.time())
        return _cb

    node.create_subscription(Imu, args.imu_topic, cb('imu'), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, args.color_info_topic, cb('color'),
                             qos_profile_sensor_data)
    node.create_subscription(CameraInfo, args.depth_info_topic, cb('depth'),
                             qos_profile_sensor_data)

    print(f'collecting {args.duration:.0f}s ...', flush=True)
    end = time.monotonic() + args.duration
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

    stats = {k: stream_stats(*v) for k, v in data.items()}
    for k in ('imu', 'color', 'depth'):
        print(fmt(k, stats[k]))

    offs = pairing_offsets(data['color'][0], data['depth'][0])
    if offs:
        offs_sorted = sorted(offs)
        p95 = offs_sorted[int(0.95 * (len(offs_sorted) - 1))]
        print(f'color↔depth nearest offset: median={statistics.median(offs) * 1e3:.1f}ms '
              f'p95={p95 * 1e3:.1f}ms max={max(offs) * 1e3:.1f}ms '
              '(rgbd_sync approx_sync_max_interval should sit just above p95)')

    ok, reasons = verdict(stats['imu'], stats['color'])
    print('VERDICT:', 'tight coupling viable' if ok else 'NOT ready for tight coupling')
    for r in reasons:
        print('  -', r)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
