#!/usr/bin/env bash
# Opens a SUMO scenario in sumo-gui (window appears on the Windows desktop via WSLg).
#   scripts/view_sumo.sh [density] [variant]      e.g. scripts/view_sumo.sh medium 5
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

DENSITY="${1:-medium}"
VARIANT="${2:-5}"
CFG="$REPO/scratch/sumo-scenario/$DENSITY/scenario_$VARIANT.sumocfg"
[[ -f "$CFG" ]] || { echo "no such scenario: $CFG" >&2; exit 1; }

echo "sumo-gui: $CFG"
exec sumo-gui -c "$CFG" --fcd-output /tmp/sumo_view_fcd.xml --delay 100 --start
