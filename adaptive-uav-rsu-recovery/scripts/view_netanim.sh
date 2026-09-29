#!/usr/bin/env bash
# Starts the NetAnim viewer (window appears on the Windows desktop via WSLg).
# NetAnim has no command-line file argument: open the printed file with the
# folder icon ("Open XML trace file") and press Play.
#   scripts/view_netanim.sh [animation.xml]
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

FILE="${1:-results/demo/rl-seed3/uav-rsu-rl-animation.xml}"
[[ "$FILE" = /* ]] || FILE="$REPO/$FILE"
[[ -f "$FILE" ]] || echo "warning: $FILE does not exist yet (run the demo simulation first)" >&2

echo "Open this file in NetAnim: $FILE"
cd "$(dirname "$FILE")"
exec netanim 2>/dev/null
