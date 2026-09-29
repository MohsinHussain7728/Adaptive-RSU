#ifndef UAVRSU_NETWORK_MONITOR_H
#define UAVRSU_NETWORK_MONITOR_H

// ENVIRONMENT LAYER - NETWORK MEASUREMENT.
// Tracks every packet of the monitored source->destination flow (UDP echo
// request and its echo reply) so that performance can be computed for any
// time window or phase instead of relying on cumulative PDR only. Also counts
//  - data packets FORWARDED by each node (proves which relay carries the flow:
//    the bridge RSU before failure, the UAV after recovery), and
//  - AODV control packets transmitted (routing overhead).
// Algorithm-independent: it knows nothing about RL.

#include "ns3/application.h"
#include "ns3/ipv4-header.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-mpdu.h"
#include "ns3/wifi-phy-common.h"
#include "ns3/node-container.h"
#include "ns3/packet.h"

#include <map>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace uavrsu
{

struct WindowStats
{
    double tStart = 0;
    double tEnd = 0;
    uint32_t tx = 0;
    uint32_t rx = 0;     // delivered within the deadline
    uint32_t echoRx = 0; // echo replies back at the source within the deadline
    double pdr = 0;      // rx / tx (0 if tx == 0)
    double loss = 1;     // 1 - pdr
    double meanDelay = 0;      // s, over delivered packets
    double throughputKbps = 0; // delivered payload over the window length

    bool Valid() const
    {
        return tx > 0;
    }
};

struct PhaseStats
{
    std::string phase;
    double duration = 0;
    uint64_t tx = 0;
    uint64_t rx = 0;       // eventually delivered
    uint64_t rxTimely = 0; // delivered within deliveryGrace
    double pdr = 0;
    double timelyPdr = 0;
    double loss = 0;
    double throughputKbps = 0;
    double meanDelayMs = 0;
    double p95DelayMs = 0;
    double jitterMs = 0;
    uint64_t aodvPackets = 0;
    uint64_t aodvBytes = 0;
    double normalizedRoutingLoad = 0; // AODV packets per delivered data packet
    uint64_t uavForwarded = 0;
    uint64_t failedRsuForwarded = 0;
};

class NetworkMonitor
{
  public:
    void Configure(uint32_t packetSize, double deliveryGrace);
    void AttachApplications(ns3::Ptr<ns3::Application> client, ns3::Ptr<ns3::Application> server);
    void AttachNodes(const ns3::NodeContainer& nodes);

    /** Marks the start of a new phase at the current simulation time. */
    void SetPhase(const std::string& phase);
    std::string GetPhase() const;

    /** Packets SENT in [tStart, tEnd) that arrived no later than deadline. */
    WindowStats GetWindow(double tStart, double tEnd, double deadline) const;

    /** Data packets forwarded by a node in [t0, t1). */
    uint64_t GetForwarded(uint32_t nodeId, double t0, double t1) const;
    /** Time of the first data packet forwarded by a node at or after t0 (-1 if none). */
    double GetFirstForwardTime(uint32_t nodeId, double t0) const;
    uint64_t GetAodvPackets(double t0, double t1) const;

    uint64_t GetTotalTx() const;
    uint64_t GetTotalRx() const;

    /**
     * Phase-based statistics. Phase intervals come from SetPhase(); extra
     * named intervals (e.g. "post_failure") can be appended.
     */
    std::vector<PhaseStats> ComputePhaseStats(
        double simEnd,
        uint32_t uavNodeId,
        int32_t failedRsuNodeId,
        const std::vector<std::pair<std::string, std::pair<double, double>>>& extra) const;

    /** Per-node drop/forward counters of the monitored flow plus Wi-Fi MAC/PHY drops (debugging aid). */
    void PrintDiagnostics(std::ostream& os) const;

  private:
    struct PacketRecord
    {
        double tx = 0;
        double rx = -1;
        double echo = -1;
    };

    void OnClientTx(ns3::Ptr<const ns3::Packet> p);
    void OnClientRx(ns3::Ptr<const ns3::Packet> p);
    void OnServerRx(ns3::Ptr<const ns3::Packet> p);
    static void ForwardSink(NetworkMonitor* self,
                            uint32_t nodeId,
                            const ns3::Ipv4Header& header,
                            ns3::Ptr<const ns3::Packet> packet,
                            uint32_t iface);
    static void TxSink(NetworkMonitor* self,
                       ns3::Ptr<const ns3::Packet> packet,
                       ns3::Ptr<ns3::Ipv4> ipv4,
                       uint32_t iface);
    static void IpDropSink(NetworkMonitor* self,
                           uint32_t nodeId,
                           const ns3::Ipv4Header& header,
                           ns3::Ptr<const ns3::Packet> packet,
                           ns3::Ipv4L3Protocol::DropReason reason,
                           ns3::Ptr<ns3::Ipv4> ipv4,
                           uint32_t iface);
    static void LocalDeliverSink(NetworkMonitor* self,
                                 uint32_t nodeId,
                                 const ns3::Ipv4Header& header,
                                 ns3::Ptr<const ns3::Packet> packet,
                                 uint32_t iface);
    static void MacDropSink(NetworkMonitor* self,
                            uint32_t nodeId,
                            ns3::WifiMacDropReason reason,
                            ns3::Ptr<const ns3::WifiMpdu> mpdu);
    static void PhyRxDropSink(NetworkMonitor* self,
                              uint32_t nodeId,
                              ns3::Ptr<const ns3::Packet> packet,
                              ns3::WifiPhyRxfailureReason reason);

    PhaseStats Aggregate(const std::string& name,
                         const std::vector<std::pair<double, double>>& intervals,
                         uint32_t uavNodeId,
                         int32_t failedRsuNodeId) const;

    uint32_t m_packetSize = 1024;
    double m_grace = 0.2;
    std::vector<PacketRecord> m_packets;
    std::unordered_map<uint64_t, std::size_t> m_uidIndex;
    std::vector<std::pair<double, std::string>> m_phaseLog;
    std::map<uint32_t, std::vector<double>> m_forwardTimes;
    std::vector<std::pair<double, uint32_t>> m_aodvTx;
    std::map<std::pair<uint32_t, int>, uint64_t> m_dataIpDrops; // (node, Ipv4L3Protocol::DropReason)
    std::map<std::pair<uint32_t, int>, uint64_t> m_macDrops; // (node, WifiMacDropReason)
    std::map<int, uint64_t> m_phyRxDrops; // WifiPhyRxfailureReason
    std::map<uint32_t, uint64_t> m_localDeliver;
    std::vector<uint64_t> m_diagUids;
};

} // namespace uavrsu

#endif
