#!/bin/bash
# Soft-replug the Orbbec Gemini camera: toggle USB `authorized` (equivalent
# to unplug/replug at the protocol level, one step stronger than usbreset).
# Installed to /usr/local/sbin/ and whitelisted in sudoers (NOPASSWD) so
# camera_watchdog (user pi) can escalate when plain usbreset fails —
# observed 2026-07-12: driver stuck in "openUsbDevice failed" loop that
# usbreset alone could not clear.
#
# Install (once):
#   sudo cp scripts/gemini-usb-replug.sh /usr/local/sbin/gemini-usb-replug
#   sudo chmod 755 /usr/local/sbin/gemini-usb-replug
#   echo 'pi ALL=(root) NOPASSWD: /usr/local/sbin/gemini-usb-replug' | \
#     sudo tee /etc/sudoers.d/gemini-replug
for d in /sys/bus/usb/devices/*/idVendor; do
    if [ "$(cat "$d" 2>/dev/null)" = "2bc5" ]; then
        dev=$(dirname "$d")
        echo 0 > "$dev/authorized"
        sleep 3
        echo 1 > "$dev/authorized"
        echo "replugged $dev"
    fi
done
