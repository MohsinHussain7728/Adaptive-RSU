#ifndef UAVRSU_ENV_SNAPSHOT_H
#define UAVRSU_ENV_SNAPSHOT_H

// ENVIRONMENT LAYER - raw observation of the network at one decision instant.
// This is the "RL environment state" before encoding. The StateEncoder turns
// it into an algorithm-facing Observation, the RewardFunction compares two
// consecutive snapshots, and non-RL strategies read it directly.

#include "network-monitor.h"
#include "topology-observer.h"

#include "ns3/vector.h"

#include <string>

namespace uavrsu
{

struct EnvSnapshot
{
    double time = 0;
    ns3::Vector uavPos;
    bool uavDeployed = true;

    WindowStats window;    // measured traffic performance in the last window
    ConnectivityInfo conn; // geometric connectivity estimate

    // Routing verification (from the source's AODV routing table)
    bool srcRouteValid = false;
    uint32_t srcRouteHops = 0;
    std::string srcRouteNextHop = "-";
    uint64_t uavForwardedWindow = 0; // data packets relayed by the UAV in the window

    bool hasFailedRsu = false;
    ns3::Vector failedRsuPos;
};

} // namespace uavrsu

#endif
