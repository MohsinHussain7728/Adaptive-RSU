#!/usr/bin/env bash
# Runs a Python script with the lab virtualenv from the repository root.
#   scripts/py.sh scripts/experiments/train.py --seeds 1,2,3
set -euo pipefail
source "$(dirname "$0")/common.sh"
cd "$REPO"
export SIM_BIN
exec "$PYTHON" "$@"
