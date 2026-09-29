#ifndef UAVRSU_TOPOLOGY_OBSERVER_H
#define UAVRSU_TOPOLOGY_OBSERVER_H

// ENVIRONMENT LAYER - GEOMETRIC CONNECTIVITY OBSERVATION.
// Builds a unit-disk connectivity graph from node positions (in a real system
// the UAV learns these from periodic vehicle beacons / CAMs and RSU status
// messages) and answers questions the recovery controller needs:
//   - is the source still connected to the destination over the ground network?
//   - is the UAV within range of the source-side / destination-side component?
//   - in which direction and how far are those components?
//   - where is the narrowest gap between the two components?
// This is only an ESTIMATE used for the state/reward; actual recovery is always
// verified with real packet delivery, forwarding counters and AODV tables.
// Algorithm-independent.

#include "ns3/node-container.h"
#include "ns3/vector.h"

#include <vector>

namespace uavrsu
{

class RsuManager;

struct ConnectivityInfo
{
    bool groundConnected = false; // src-dst path without the UAV
    bool e2eConnected = false;    // src-dst path including the UAV
    bool uavLinkSrc = false;      // UAV adjacent to the source-side ground component
    bool uavLinkDst = false;      // UAV adjacent to the destination-side ground component
    double distSrcComp = 0;       // m, UAV to nearest node of the source-side component
    double distDstComp = 0;
    double bearingSrc = 0; // rad, horizontal direction UAV -> that node (0 = East, pi/2 = North)
    double bearingDst = 0;
    bool gapValid = false; // closest pair between the two components (only if partitioned)
    ns3::Vector gapMidpoint;
    double gapDistance = 0;
    uint32_t srcCompSize = 0;
    uint32_t dstCompSize = 0;
    uint32_t uavNeighbors = 0;
};

class TopologyObserver
{
  public:
    void Configure(double linkRange);
    void SetNodes(const ns3::NodeContainer& vehicles,
                  const RsuManager* rsus,
                  ns3::Ptr<ns3::Node> uav,
                  uint32_t sourceIndex,
                  uint32_t destinationIndex);

    ConnectivityInfo Observe(bool uavDeployed) const;
    double GetLinkRange() const;

  private:
    double m_range = 237.5;
    ns3::NodeContainer m_vehicles;
    const RsuManager* m_rsus = nullptr;
    ns3::Ptr<ns3::Node> m_uav;
    uint32_t m_src = 0;
    uint32_t m_dst = 0;
};

} // namespace uavrsu

#endif
