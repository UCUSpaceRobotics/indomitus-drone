#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_PY="$REPO_ROOT/.venv/bin/python"

# The desktop-session user that owns the active display.
TARGET_USER="${TARGET_USER:-erso}"

# X11 bridge variables needed by Qt/OpenCV when launching from SSH.
DISPLAY_NAME="${DISPLAY_NAME:-:0}"
XAUTHORITY_FILE="${XAUTHORITY_FILE:-/home/erso/.Xauthority}"

# Wayland session environment for Raspberry Pi OS.
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

exec sudo -u "$TARGET_USER" env \
  DISPLAY="$DISPLAY_NAME" \
  XAUTHORITY="$XAUTHORITY_FILE" \
  XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
  QT_QPA_PLATFORM="$QT_QPA_PLATFORM" \
  "$VENV_PY" \
  "$REPO_ROOT/scripts/test_composite_output.py" \
  --backend picamera2 \
  --device-index 0