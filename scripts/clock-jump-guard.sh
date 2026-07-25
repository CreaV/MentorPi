#!/bin/bash
# Restart the MentorPi stack when the system clock steps underneath it.
#
# The Pi has no RTC battery: fake-hwclock boots ROS on the time saved at
# last shutdown, then NTP steps the clock once WiFi comes up (observed
# +22h mid-run 2026-07-17) — EKF hallucinates, TF buffers empty out,
# topics go silent, the camera pipeline dies. None of it logs an error.
#
# Detection: over each interval, CLOCK_REALTIME advance minus
# CLOCK_BOOTTIME advance ≈ 0 on a healthy clock; a step shows up as a
# large divergence. Restarting mentorpi-remote is safe — the supervisor
# gives rtabmap a 90 s SIGINT grace to flush its database.
set -u

THRESHOLD=10   # seconds of divergence that counts as a step
INTERVAL=5     # seconds between checks

prev_rt=$(date +%s)
prev_bt=$(cut -d. -f1 /proc/uptime)

while sleep "$INTERVAL"; do
    rt=$(date +%s)
    bt=$(cut -d. -f1 /proc/uptime)
    drift=$(( (rt - prev_rt) - (bt - prev_bt) ))
    prev_rt=$rt
    prev_bt=$bt
    if [ "${drift#-}" -gt "$THRESHOLD" ]; then
        echo "system clock stepped ${drift}s — restarting mentorpi-remote"
        systemctl try-restart mentorpi-remote.service
    fi
done
