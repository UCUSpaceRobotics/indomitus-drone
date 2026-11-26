#!/usr/bin/env bash
set -e

SCRIPT=$(readlink -f "$0")
ROOT="$(dirname "$(dirname "$SCRIPT")")"

# Container already running: open another shell in it instead of starting a second one
if [ -n "$(docker ps -q -f name=^indomitus-drone$)" ]; then
  exec docker exec -it indomitus-drone bash
fi

# GPU detection: use the NVIDIA GPU only if it is actually usable, otherwise fall back to CPU
GPU_ARGS=()
if docker info 2>/dev/null | grep -qi "nvidia"; then
  # Linux with NVIDIA Container Toolkit installed
  GPU_ARGS=(--gpus all)
elif grep -qi microsoft /proc/version 2>/dev/null && command -v nvidia-smi >/dev/null && nvidia-smi >/dev/null 2>&1; then
  # WSL2 (Docker Desktop) with an NVIDIA Windows driver
  GPU_ARGS=(--gpus all)
fi

if [ ${#GPU_ARGS[@]} -gt 0 ]; then
  echo "NVIDIA GPU detected: enabling GPU rendering"
else
  echo "No usable NVIDIA GPU detected: using CPU rendering"
fi


exec docker run -it --rm --init \
  "${GPU_ARGS[@]}" \
  --name indomitus-drone \
  -p 8765:8765 \
  -v "$ROOT:/workspace" \
  -v "$(pwd)/docker/.tmux.conf:/root/.tmux.conf:ro" \
  indomitus-drone:dev bash