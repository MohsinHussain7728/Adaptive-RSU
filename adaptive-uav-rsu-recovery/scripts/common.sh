#!/usr/bin/env bash
# Shared paths for all experiment scripts (run inside the ns3-uav-lab WSL distro).
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NS3_HOME="${NS3_HOME:-/opt/ns3-uav-lab/ns-allinone-3.47/ns-3.47}"
SIM_BIN="$NS3_HOME/build/scratch/uav-rsu-rl/ns3.47-uav-rsu-rl-optimized"
RL_TEST_BIN="$NS3_HOME/build/scratch/uav-rsu-rl/uav-rl-agent-test"
PYTHON="${PYTHON:-/opt/ns3-uav-lab/venv/bin/python}"
export SUMO_HOME="${SUMO_HOME:-/usr/share/sumo}"
