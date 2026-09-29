#!/usr/bin/env bash
# Regenerates SUMO scenarios. No argument: all scenarios (generate_all.sh);
# otherwise the arguments are passed to generate_scenario.py.
set -euo pipefail
source "$(dirname "$0")/common.sh"
cd "$REPO/scratch/sumo-scenario"
if [[ $# -eq 0 ]]; then
    bash generate_all.sh
else
    python3 generate_scenario.py "$@"
fi
