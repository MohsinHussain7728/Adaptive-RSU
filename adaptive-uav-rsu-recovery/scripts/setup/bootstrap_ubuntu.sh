#!/usr/bin/env bash
# Runs INSIDE the dedicated WSL distro (as root). Installs:
#   - build tools (g++, cmake, ninja), rsync
#   - SUMO 1.18.0 (Ubuntu 24.04 package) + SUMO tools
#   - Python venv with numpy/pandas/matplotlib/tabulate for analysis
#   - ns-3.47 built with only the modules this project needs (optimized profile)
#
# Everything lives in the distro's virtual disk, i.e. inside C:\ns3-uav-lab.
set -euo pipefail

JOBS="${1:-8}"
LAB=/opt/ns3-uav-lab
NS3_VERSION=3.47
NS3_DIR="$LAB/ns-allinone-$NS3_VERSION/ns-$NS3_VERSION"

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
    build-essential g++ cmake ninja-build git rsync wget ca-certificates bzip2 \
    python3 python3-venv python3-dev python-is-python3 \
    sumo sumo-tools

mkdir -p "$LAB"

if [ ! -d "$LAB/venv" ]; then
    python3 -m venv --system-site-packages "$LAB/venv"
fi
"$LAB/venv/bin/pip" install --quiet --upgrade pip
"$LAB/venv/bin/pip" install --quiet numpy pandas matplotlib tabulate

if [ ! -d "$NS3_DIR" ]; then
    cd "$LAB"
    wget -q "https://www.nsnam.org/releases/ns-allinone-$NS3_VERSION.tar.bz2"
    tar -xjf "ns-allinone-$NS3_VERSION.tar.bz2"
    rm -f "ns-allinone-$NS3_VERSION.tar.bz2"
fi

# ns-3 refuses to build as root: use an unprivileged default user.
id -u researcher >/dev/null 2>&1 || useradd -m -s /bin/bash researcher
printf '[user]\ndefault=researcher\n' > /etc/wsl.conf
chown -R researcher:researcher "$LAB"

su researcher -c "cd '$NS3_DIR' && ./ns3 configure -d optimized \
    --disable-examples --disable-tests --disable-python \
    --enable-modules='core;network;mobility;wifi;internet;aodv;applications;flow-monitor;netanim' \
    && ./ns3 build -j $JOBS"

cat > /etc/profile.d/ns3-uav-lab.sh <<EOF
export NS3_HOME=$NS3_DIR
export SUMO_HOME=/usr/share/sumo
export PATH=$LAB/venv/bin:\$PATH
EOF

echo "BOOTSTRAP_DONE"
sumo --version | head -1
echo "NS3_HOME=$NS3_DIR"
