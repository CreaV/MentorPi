#!/usr/bin/env python3
"""Unit tests for the pure statistics in check_camera_imu_sync — no ROS.

    python3 -m pytest scripts/tests/test_check_camera_imu_sync.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import check_camera_imu_sync as c  # noqa: E402  (scripts/ on sys.path)


def _stream(rate, n, t0=1000.0, latency=0.01, jitter=0.0):
    stamps = [t0 + i / rate + (jitter if i % 2 else 0.0) for i in range(n)]
    recv = [s + latency for s in stamps]
    return stamps, recv


def test_stream_stats_rate_and_latency():
    s = c.stream_stats(*_stream(200.0, 401, latency=0.002))
    assert abs(s['rate_hz'] - 200.0) < 1e-6
    assert abs(s['dt_mean'] - 0.005) < 1e-9
    assert s['dt_std'] < 1e-9
    assert s['non_monotonic'] == 0
    assert abs(s['lat_mean'] - 0.002) < 1e-9


def test_stream_stats_counts_non_monotonic():
    stamps = [1.0, 2.0, 1.5, 3.0]
    s = c.stream_stats(stamps, [x + 0.01 for x in stamps])
    assert s['non_monotonic'] == 1


def test_stream_stats_too_short():
    assert c.stream_stats([1.0], [1.0]) == {'count': 1}


def test_pairing_offsets_nearest():
    color = [0.000, 0.066, 0.133]
    depth = [0.010, 0.070, 0.150]
    offs = c.pairing_offsets(color, depth)
    assert [round(o, 3) for o in offs] == [0.010, 0.004, 0.017]
    assert c.pairing_offsets(color, []) == []


def test_verdict_ok():
    imu = c.stream_stats(*_stream(200.0, 400, latency=0.002))
    color = c.stream_stats(*_stream(15.0, 30, latency=0.06))
    ok, reasons = c.verdict(imu, color)
    assert ok, reasons


def test_verdict_device_clock_domain():
    # IMU stamped with a raw device clock: ~1e5 s away from host time.
    imu = c.stream_stats(*_stream(200.0, 400, latency=-1e5))
    color = c.stream_stats(*_stream(15.0, 30, latency=0.06))
    ok, reasons = c.verdict(imu, color)
    assert not ok
    assert any('clock domain' in r for r in reasons)


def test_verdict_jitter():
    imu = c.stream_stats(*_stream(200.0, 400, jitter=0.002))
    color = c.stream_stats(*_stream(15.0, 30, latency=0.06))
    ok, reasons = c.verdict(imu, color)
    assert not ok
    assert any('jitter' in r for r in reasons)


def test_verdict_missing_imu():
    ok, reasons = c.verdict({'count': 0}, {'count': 0})
    assert not ok
    assert 'no camera IMU' in reasons[0]
