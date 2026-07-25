#!/bin/bash
# Install / refresh the mentorpi-remote systemd unit and enable boot autostart.
# Usage:
#   bash scripts/install-systemd.sh          # install + start now + enable on boot
#   bash scripts/install-systemd.sh disable  # stop + disable + uninstall
set -euo pipefail

UNIT_NAME=mentorpi-remote.service
GUARD_UNIT=mentorpi-clockguard.service
GUARD_SCRIPT=clock-jump-guard.sh
SRC_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC_UNIT="$SRC_DIR/$UNIT_NAME"
DST_UNIT="/etc/systemd/system/$UNIT_NAME"

case "${1:-install}" in
  install)
    [ -f "$SRC_UNIT" ] || { echo "Missing $SRC_UNIT"; exit 1; }
    sudo install -m 644 "$SRC_UNIT" "$DST_UNIT"
    sudo install -m 755 "$SRC_DIR/$GUARD_SCRIPT" "/usr/local/sbin/$GUARD_SCRIPT"
    sudo install -m 644 "$SRC_DIR/$GUARD_UNIT" "/etc/systemd/system/$GUARD_UNIT"
    sudo systemctl daemon-reload
    sudo systemctl enable --now "$GUARD_UNIT"
    sudo systemctl enable --now "$UNIT_NAME"
    echo
    sudo systemctl --no-pager --full status "$UNIT_NAME" || true
    echo
    echo "Tail live logs:  journalctl -u $UNIT_NAME -f"
    ;;
  disable)
    sudo systemctl disable --now "$UNIT_NAME" || true
    sudo systemctl disable --now "$GUARD_UNIT" || true
    sudo rm -f "$DST_UNIT" "/etc/systemd/system/$GUARD_UNIT" "/usr/local/sbin/$GUARD_SCRIPT"
    sudo systemctl daemon-reload
    echo "Disabled and removed $UNIT_NAME + $GUARD_UNIT"
    ;;
  *)
    echo "Usage: $0 [install|disable]" >&2
    exit 2
    ;;
esac
