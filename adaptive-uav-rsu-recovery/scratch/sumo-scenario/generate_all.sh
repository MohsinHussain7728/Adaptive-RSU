#!/usr/bin/env bash
# Regenerates every SUMO scenario used in the study (deterministic seeds).
#   low / medium / high   : 20 / 40 / 60 vehicles at 50 km/h
#   medium_slow / _fast   : 40 vehicles at 30 km/h / 70 km/h (speed sensitivity)
# Each scenario has 5 mobility variants (SUMO seeds 1..5).
set -euo pipefail
cd "$(dirname "$0")"

python3 generate_scenario.py --name low         --vehicles 20 --max-speed 13.89
python3 generate_scenario.py --name medium      --vehicles 40 --max-speed 13.89
python3 generate_scenario.py --name high        --vehicles 60 --max-speed 13.89
python3 generate_scenario.py --name medium_slow --vehicles 40 --max-speed 8.33
python3 generate_scenario.py --name medium_fast --vehicles 40 --max-speed 19.44
