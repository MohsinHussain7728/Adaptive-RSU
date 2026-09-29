#!/usr/bin/env bash
# Copies scratch/uav-rsu-rl into the ns-3 tree and builds it (plus the RL unit test).
set -euo pipefail
source "$(dirname "$0")/common.sh"

rsync -a --delete "$REPO/scratch/uav-rsu-rl/" "$NS3_HOME/scratch/uav-rsu-rl/"
cd "$NS3_HOME"
if [[ ! -f cmake-cache/build.ninja && ! -f cmake-cache/Makefile ]]; then
    ./ns3 configure
fi
cmake --build cmake-cache -j "${JOBS:-$(nproc)}" --target scratch_uav-rsu-rl uav-rl-agent-test
echo "built: $SIM_BIN"
