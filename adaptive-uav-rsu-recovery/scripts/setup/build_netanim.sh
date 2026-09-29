#!/usr/bin/env bash
# Runs INSIDE the dedicated WSL distro (as root). Builds the NetAnim viewer
# (Qt5) into /opt/ns3-uav-lab/netanim and links it as /usr/local/bin/netanim.
# The window opens on the Windows desktop through WSLg.
set -euo pipefail

JOBS="${1:-8}"
LAB=/opt/ns3-uav-lab
DIR="$LAB/netanim"

export DEBIAN_FRONTEND=noninteractive
apt-get install -y --no-install-recommends qtbase5-dev qtbase5-dev-tools libqt5svg5-dev cmake g++ git

if [ ! -d "$DIR" ]; then
    git clone --depth 1 https://gitlab.com/nsnam/netanim.git "$DIR"
fi
chown -R researcher:researcher "$DIR"
su researcher -c "cd '$DIR' && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j $JOBS"
BIN="$(find "$DIR/build" -maxdepth 3 -type f -iname netanim -perm -u+x | head -1)"
ln -sf "$BIN" /usr/local/bin/netanim
echo "NETANIM_DONE: $(command -v netanim)"
