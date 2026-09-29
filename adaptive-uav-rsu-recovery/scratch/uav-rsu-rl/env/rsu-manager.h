#ifndef UAVRSU_RSU_MANAGER_H
#define UAVRSU_RSU_MANAGER_H

// ENVIRONMENT LAYER - RSU infrastructure and the failure mechanism.
// An RSU failure switches the RSU's Wi-Fi PHY off, so it stops transmitting,
// receiving and forwarding. AODV then observes broken links exactly as it would
// for a real power/hardware failure. Algorithm-independent.

#include "ns3/callback.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/vector.h"

#include <string>
#include <vector>

namespace uavrsu
{

struct RsuInfo
{
    uint32_t id = 0;
    ns3::Ptr<ns3::Node> node;
    ns3::Vector position;
    bool active = true;
    double failureTime = -1.0;
};

class RsuManager
{
  public:
    void Create(const std::vector<ns3::Vector>& positions);
    void InstallMobility();

    ns3::NodeContainer GetNodes() const;
    std::vector<RsuInfo>& GetRsus();
    const std::vector<RsuInfo>& GetRsus() const;
    uint32_t GetN() const;
    bool IsActive(uint32_t id) const;

    /** Switches the RSU radio off and notifies listeners. */
    void FailRsu(uint32_t id);

    /** Listener invoked after an RSU fails (argument: RSU id). */
    void AddFailureListener(ns3::Callback<void, uint32_t> cb);

    void Print(std::ostream& os) const;

  private:
    ns3::NodeContainer m_nodes;
    std::vector<RsuInfo> m_rsus;
    std::vector<ns3::Callback<void, uint32_t>> m_listeners;
};

} // namespace uavrsu

#endif
