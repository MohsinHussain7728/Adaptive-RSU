#include "network-monitor.h"

#include "ns3/callback.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/simulator.h"
#include "ns3/udp-header.h"
#include "ns3/udp-l4-protocol.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy.h"

#include <algorithm>
#include <cmath>

namespace uavrsu
{

using namespace ns3;

namespace
{
constexpr uint16_t kDataPort = 9;   // UDP echo
constexpr uint16_t kAodvPort = 654; // AODV control

uint64_t
CountInRange(const std::vector<double>& times, double t0, double t1)
{
    auto a = std::lower_bound(times.begin(), times.end(), t0);
    auto b = std::lower_bound(times.begin(), times.end(), t1);
    return static_cast<uint64_t>(b - a);
}
} // namespace

void
NetworkMonitor::Configure(uint32_t packetSize, double deliveryGrace)
{
    m_packetSize = packetSize;
    m_grace = deliveryGrace;
}

void
NetworkMonitor::AttachApplications(Ptr<Application> client, Ptr<Application> server)
{
    client->TraceConnectWithoutContext("Tx", MakeCallback(&NetworkMonitor::OnClientTx, this));
    client->TraceConnectWithoutContext("Rx", MakeCallback(&NetworkMonitor::OnClientRx, this));
    // ns-3.47: UdpEchoServer is a SinkApplication whose "Rx" also carries the sender address.
    server->TraceConnectWithoutContext("RxWithoutAddress", MakeCallback(&NetworkMonitor::OnServerRx, this));
}

void
NetworkMonitor::AttachNodes(const NodeContainer& nodes)
{
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        Ptr<Node> n = nodes.Get(i);
        Ptr<Ipv4L3Protocol> ip = n->GetObject<Ipv4L3Protocol>();
        if (!ip)
        {
            continue;
        }
        ip->TraceConnectWithoutContext("UnicastForward",
                                       MakeBoundCallback(&NetworkMonitor::ForwardSink, this, n->GetId()));
        ip->TraceConnectWithoutContext("Tx", MakeBoundCallback(&NetworkMonitor::TxSink, this));
        ip->TraceConnectWithoutContext("Drop", MakeBoundCallback(&NetworkMonitor::IpDropSink, this, n->GetId()));
        ip->TraceConnectWithoutContext("LocalDeliver",
                                       MakeBoundCallback(&NetworkMonitor::LocalDeliverSink, this, n->GetId()));
        for (uint32_t d = 0; d < n->GetNDevices(); ++d)
        {
            Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(n->GetDevice(d));
            if (!dev)
            {
                continue;
            }
            dev->GetMac()->TraceConnectWithoutContext(
                "DroppedMpdu",
                MakeBoundCallback(&NetworkMonitor::MacDropSink, this, n->GetId()));
            dev->GetPhy()->TraceConnectWithoutContext(
                "PhyRxDrop",
                MakeBoundCallback(&NetworkMonitor::PhyRxDropSink, this, n->GetId()));
        }
    }
}

void
NetworkMonitor::IpDropSink(NetworkMonitor* self,
                           uint32_t nodeId,
                           const Ipv4Header& header,
                           Ptr<const Packet> packet,
                           Ipv4L3Protocol::DropReason reason,
                           Ptr<Ipv4> /*ipv4*/,
                           uint32_t /*iface*/)
{
    if (header.GetProtocol() != UdpL4Protocol::PROT_NUMBER)
    {
        return;
    }
    UdpHeader udp;
    packet->PeekHeader(udp);
    if (udp.GetDestinationPort() == kDataPort || udp.GetSourcePort() == kDataPort)
    {
        self->m_dataIpDrops[{nodeId, static_cast<int>(reason)}]++;
    }
}

void
NetworkMonitor::LocalDeliverSink(NetworkMonitor* self,
                                 uint32_t nodeId,
                                 const Ipv4Header& header,
                                 Ptr<const Packet> packet,
                                 uint32_t /*iface*/)
{
    if (header.GetProtocol() != UdpL4Protocol::PROT_NUMBER)
    {
        return;
    }
    UdpHeader udp;
    packet->PeekHeader(udp);
    if (udp.GetDestinationPort() == kDataPort || udp.GetSourcePort() == kDataPort)
    {
        self->m_localDeliver[nodeId]++;
        if (self->m_diagUids.size() < 3)
        {
            self->m_diagUids.push_back(packet->GetUid());
        }
    }
}

void
NetworkMonitor::MacDropSink(NetworkMonitor* self,
                            uint32_t nodeId,
                            WifiMacDropReason reason,
                            Ptr<const WifiMpdu> /*mpdu*/)
{
    self->m_macDrops[{nodeId, static_cast<int>(reason)}]++;
}

void
NetworkMonitor::PhyRxDropSink(NetworkMonitor* self,
                              uint32_t /*nodeId*/,
                              Ptr<const Packet> /*packet*/,
                              WifiPhyRxfailureReason reason)
{
    self->m_phyRxDrops[static_cast<int>(reason)]++;
}

void
NetworkMonitor::PrintDiagnostics(std::ostream& os) const
{
    os << "--- diagnostics: data-flow forwarding per node ---\n";
    for (const auto& [node, times] : m_forwardTimes)
    {
        os << "  node " << node << " forwarded " << times.size() << "\n";
    }
    os << "--- data-flow IPv4 local deliveries per node ---\n";
    for (const auto& [node, n] : m_localDeliver)
    {
        os << "  node " << node << " delivered " << n << "\n";
    }
    os << "  first delivered UIDs:";
    for (auto u : m_diagUids)
    {
        os << " " << u;
    }
    os << " | first sent UIDs:";
    for (const auto& [uid, idx] : m_uidIndex)
    {
        if (idx < 3)
        {
            os << " " << uid;
        }
    }
    os << "\n";
    os << "--- data-flow IPv4 drops (node, Ipv4L3Protocol::DropReason) ---\n";
    for (const auto& [key, n] : m_dataIpDrops)
    {
        os << "  node " << key.first << " reason " << key.second << " : " << n << "\n";
    }
    os << "--- Wi-Fi MAC drops, all frames (node, WifiMacDropReason) ---\n";
    for (const auto& [key, n] : m_macDrops)
    {
        os << "  node " << key.first << " reason " << key.second << " : " << n << "\n";
    }
    os << "--- Wi-Fi PHY RX drops, all frames (WifiPhyRxfailureReason) ---\n";
    for (const auto& [reason, n] : m_phyRxDrops)
    {
        os << "  reason " << reason << " : " << n << "\n";
    }
}

void
NetworkMonitor::SetPhase(const std::string& phase)
{
    double now = Simulator::Now().GetSeconds();
    if (!m_phaseLog.empty() && m_phaseLog.back().second == phase)
    {
        return;
    }
    m_phaseLog.emplace_back(now, phase);
}

std::string
NetworkMonitor::GetPhase() const
{
    return m_phaseLog.empty() ? "startup" : m_phaseLog.back().second;
}

void
NetworkMonitor::OnClientTx(Ptr<const Packet> p)
{
    PacketRecord r;
    r.tx = Simulator::Now().GetSeconds();
    m_uidIndex[p->GetUid()] = m_packets.size();
    m_packets.push_back(r);
}

void
NetworkMonitor::OnServerRx(Ptr<const Packet> p)
{
    auto it = m_uidIndex.find(p->GetUid());
    if (it != m_uidIndex.end() && m_packets[it->second].rx < 0)
    {
        m_packets[it->second].rx = Simulator::Now().GetSeconds();
    }
}

void
NetworkMonitor::OnClientRx(Ptr<const Packet> p)
{
    // The echo server sends the same packet object back, so the UID matches.
    auto it = m_uidIndex.find(p->GetUid());
    if (it != m_uidIndex.end() && m_packets[it->second].echo < 0)
    {
        m_packets[it->second].echo = Simulator::Now().GetSeconds();
    }
}

void
NetworkMonitor::ForwardSink(NetworkMonitor* self,
                            uint32_t nodeId,
                            const Ipv4Header& header,
                            Ptr<const Packet> packet,
                            uint32_t /*iface*/)
{
    if (header.GetProtocol() != UdpL4Protocol::PROT_NUMBER)
    {
        return;
    }
    UdpHeader udp;
    packet->PeekHeader(udp);
    if (udp.GetDestinationPort() == kDataPort || udp.GetSourcePort() == kDataPort)
    {
        self->m_forwardTimes[nodeId].push_back(Simulator::Now().GetSeconds());
    }
}

void
NetworkMonitor::TxSink(NetworkMonitor* self, Ptr<const Packet> packet, Ptr<Ipv4> /*ipv4*/, uint32_t /*iface*/)
{
    Ptr<Packet> copy = packet->Copy();
    Ipv4Header ip;
    copy->RemoveHeader(ip);
    if (ip.GetProtocol() != UdpL4Protocol::PROT_NUMBER)
    {
        return;
    }
    UdpHeader udp;
    copy->PeekHeader(udp);
    if (udp.GetDestinationPort() == kAodvPort)
    {
        self->m_aodvTx.emplace_back(Simulator::Now().GetSeconds(), packet->GetSize());
    }
}

WindowStats
NetworkMonitor::GetWindow(double tStart, double tEnd, double deadline) const
{
    WindowStats w;
    w.tStart = tStart;
    w.tEnd = tEnd;
    auto first = std::lower_bound(m_packets.begin(), m_packets.end(), tStart,
                                  [](const PacketRecord& r, double t) { return r.tx < t; });
    double delaySum = 0;
    for (auto it = first; it != m_packets.end() && it->tx < tEnd; ++it)
    {
        w.tx++;
        if (it->rx >= 0 && it->rx <= deadline)
        {
            w.rx++;
            delaySum += it->rx - it->tx;
        }
        if (it->echo >= 0 && it->echo <= deadline)
        {
            w.echoRx++;
        }
    }
    if (w.tx > 0)
    {
        w.pdr = static_cast<double>(w.rx) / w.tx;
        w.loss = 1.0 - w.pdr;
    }
    if (w.rx > 0)
    {
        w.meanDelay = delaySum / w.rx;
    }
    if (tEnd > tStart)
    {
        w.throughputKbps = w.rx * m_packetSize * 8.0 / (tEnd - tStart) / 1000.0;
    }
    return w;
}

uint64_t
NetworkMonitor::GetForwarded(uint32_t nodeId, double t0, double t1) const
{
    auto it = m_forwardTimes.find(nodeId);
    return it == m_forwardTimes.end() ? 0 : CountInRange(it->second, t0, t1);
}

double
NetworkMonitor::GetFirstForwardTime(uint32_t nodeId, double t0) const
{
    auto it = m_forwardTimes.find(nodeId);
    if (it == m_forwardTimes.end())
    {
        return -1.0;
    }
    auto f = std::lower_bound(it->second.begin(), it->second.end(), t0);
    return f == it->second.end() ? -1.0 : *f;
}

uint64_t
NetworkMonitor::GetAodvPackets(double t0, double t1) const
{
    uint64_t n = 0;
    for (const auto& e : m_aodvTx)
    {
        n += (e.first >= t0 && e.first < t1);
    }
    return n;
}

uint64_t
NetworkMonitor::GetTotalTx() const
{
    return m_packets.size();
}

uint64_t
NetworkMonitor::GetTotalRx() const
{
    uint64_t n = 0;
    for (const auto& p : m_packets)
    {
        n += (p.rx >= 0);
    }
    return n;
}

PhaseStats
NetworkMonitor::Aggregate(const std::string& name,
                          const std::vector<std::pair<double, double>>& intervals,
                          uint32_t uavNodeId,
                          int32_t failedRsuNodeId) const
{
    PhaseStats s;
    s.phase = name;
    std::vector<double> delays;
    double prevDelay = -1;
    double jitterSum = 0;
    uint64_t jitterN = 0;

    for (const auto& iv : intervals)
    {
        double t0 = iv.first;
        double t1 = iv.second;
        s.duration += t1 - t0;
        for (const auto& p : m_packets)
        {
            if (p.tx < t0 || p.tx >= t1)
            {
                continue;
            }
            s.tx++;
            if (p.rx >= 0)
            {
                double d = p.rx - p.tx;
                s.rx++;
                s.rxTimely += (d <= m_grace);
                delays.push_back(d);
                if (prevDelay >= 0)
                {
                    jitterSum += std::fabs(d - prevDelay);
                    jitterN++;
                }
                prevDelay = d;
            }
        }
        for (const auto& e : m_aodvTx)
        {
            if (e.first >= t0 && e.first < t1)
            {
                s.aodvPackets++;
                s.aodvBytes += e.second;
            }
        }
        s.uavForwarded += GetForwarded(uavNodeId, t0, t1);
        if (failedRsuNodeId >= 0)
        {
            s.failedRsuForwarded += GetForwarded(static_cast<uint32_t>(failedRsuNodeId), t0, t1);
        }
    }
    if (s.tx > 0)
    {
        s.pdr = static_cast<double>(s.rx) / s.tx;
        s.timelyPdr = static_cast<double>(s.rxTimely) / s.tx;
        s.loss = 1.0 - s.pdr;
    }
    if (s.duration > 0)
    {
        s.throughputKbps = s.rx * m_packetSize * 8.0 / s.duration / 1000.0;
    }
    if (!delays.empty())
    {
        double sum = 0;
        for (double d : delays)
        {
            sum += d;
        }
        s.meanDelayMs = 1000.0 * sum / delays.size();
        std::sort(delays.begin(), delays.end());
        s.p95DelayMs = 1000.0 * delays[static_cast<std::size_t>(0.95 * (delays.size() - 1))];
    }
    if (jitterN > 0)
    {
        s.jitterMs = 1000.0 * jitterSum / jitterN;
    }
    if (s.rx > 0)
    {
        s.normalizedRoutingLoad = static_cast<double>(s.aodvPackets) / s.rx;
    }
    return s;
}

std::vector<PhaseStats>
NetworkMonitor::ComputePhaseStats(
    double simEnd,
    uint32_t uavNodeId,
    int32_t failedRsuNodeId,
    const std::vector<std::pair<std::string, std::pair<double, double>>>& extra) const
{
    std::map<std::string, std::vector<std::pair<double, double>>> byPhase;
    std::vector<std::string> order;
    for (std::size_t i = 0; i < m_phaseLog.size(); ++i)
    {
        double t0 = m_phaseLog[i].first;
        double t1 = (i + 1 < m_phaseLog.size()) ? m_phaseLog[i + 1].first : simEnd;
        const std::string& name = m_phaseLog[i].second;
        if (t1 <= t0)
        {
            continue;
        }
        if (byPhase.find(name) == byPhase.end())
        {
            order.push_back(name);
        }
        byPhase[name].emplace_back(t0, t1);
    }
    std::vector<PhaseStats> out;
    for (const auto& name : order)
    {
        out.push_back(Aggregate(name, byPhase[name], uavNodeId, failedRsuNodeId));
    }
    for (const auto& e : extra)
    {
        if (e.second.second > e.second.first)
        {
            out.push_back(Aggregate(e.first, {e.second}, uavNodeId, failedRsuNodeId));
        }
    }
    return out;
}

} // namespace uavrsu
