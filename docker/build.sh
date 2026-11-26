#!/usr/bin/env bash
set -e

SCRIPT=$(readlink -f $0)
ROOT=$(dirname "$(dirname "$SCRIPT")")

docker build -t indomitus-drone:dev -f "$ROOT/docker/Dockerfile" "$ROOT"