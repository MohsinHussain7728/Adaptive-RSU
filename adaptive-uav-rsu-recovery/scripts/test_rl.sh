#!/usr/bin/env bash
# Runs the ns-3-independent unit test of the RL layer (rl/).
set -euo pipefail
source "$(dirname "$0")/common.sh"
tmp="$(mktemp -d)"
cd "$tmp"
"$RL_TEST_BIN"
rm -rf "$tmp"
