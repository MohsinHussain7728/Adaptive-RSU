#!/usr/bin/env bash
# Runs one simulation process from the repository root, forwarding all arguments.
#   scripts/run_sim.sh --strategy=rl --mode=train --trainingEpisodes=5 --outputDir=results/debug
set -euo pipefail
source "$(dirname "$0")/common.sh"
cd "$REPO"
exec "$SIM_BIN" "$@"
