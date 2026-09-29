#include "topology-observer.h"

#include "rsu-manager.h"

#include "ns3/mobility-model.h"

#include <cmath>
#include <limits>
#include <queue>

namespace uavrsu
{

using namespace ns3;

namespace
{

std::vector<bool>
Component(const std::vector<Vector>& pos, uint32_t start, double range)
{
    std::vector<bool> seen(pos.size(), false);
    std::queue<uint32_t> q;
    seen[start] = true;
    q.push(start);
    while (!q.empty())
    {
        uint32_t u = q.front();
        q.pop();
        for (uint32_t v = 0; v < pos.size(); ++v)
        {
            if (!seen[v] && CalculateDistance(pos[u], pos[v]) <= range)
            {
                seen[v] = true;
                q.push(v);
            }
        }
    }
    return seen;
}

} // namespace

void
TopologyObserver::Configure(double linkRange)
{
    m_range = linkRange;
}

double
TopologyObserver::GetLinkRange() const
{
    return m_range;
}

void
TopologyObserver::SetNodes(const NodeContainer& vehicles,
                           const RsuManager* rsus,
                           Ptr<Node> uav,
                           uint32_t sourceIndex,
                           uint32_t destinationIndex)
{
    m_vehicles = vehicles;
    m_rsus = rsus;
    m_uav = uav;
    m_src = sourceIndex;
    m_dst = destinationIndex;
}

ConnectivityInfo
TopologyObserver::Observe(bool uavDeployed) const
{
    ConnectivityInfo info;

    // Ground nodes = all vehicles + ACTIVE RSUs (a failed RSU is invisible).
    std::vector<Vector> pos;
    for (uint32_t i = 0; i < m_vehicles.GetN(); ++i)
    {
        pos.push_back(m_vehicles.Get(i)->GetObject<MobilityModel>()->GetPosition());
    }
    for (const auto& r : m_rsus->GetRsus())
    {
        if (r.active)
        {
            pos.push_back(r.node->GetObject<MobilityModel>()->GetPosition());
        }
    }

    auto srcComp = Component(pos, m_src, m_range);
    auto dstComp = Component(pos, m_dst, m_range);
    info.groundConnected = srcComp[m_dst];

    Vector uav = m_uav->GetObject<MobilityModel>()->GetPosition();
    double bestSrc = std::numeric_limits<double>::infinity();
    double bestDst = std::numeric_limits<double>::infinity();
    Vector nearSrc = uav;
    Vector nearDst = uav;
    for (uint32_t i = 0; i < pos.size(); ++i)
    {
        double d = CalculateDistance(uav, pos[i]);
        info.srcCompSize += srcComp[i];
        info.dstCompSize += dstComp[i];
        if (uavDeployed && d <= m_range)
        {
            info.uavNeighbors++;
        }
        if (srcComp[i] && d < bestSrc)
        {
            bestSrc = d;
            nearSrc = pos[i];
        }
        if (dstComp[i] && d < bestDst)
        {
            bestDst = d;
            nearDst = pos[i];
        }
    }
    info.distSrcComp = bestSrc;
    info.distDstComp = bestDst;
    info.bearingSrc = std::atan2(nearSrc.y - uav.y, nearSrc.x - uav.x);
    info.bearingDst = std::atan2(nearDst.y - uav.y, nearDst.x - uav.x);
    info.uavLinkSrc = uavDeployed && bestSrc <= m_range;
    info.uavLinkDst = uavDeployed && bestDst <= m_range;
    info.e2eConnected = info.groundConnected || (info.uavLinkSrc && info.uavLinkDst);

    if (!info.groundConnected)
    {
        // Narrowest gap between the two ground components: the natural relay
        // point used by the reactive (non-RL) baseline and by reward shaping.
        double best = std::numeric_limits<double>::infinity();
        for (uint32_t i = 0; i < pos.size(); ++i)
        {
            if (!srcComp[i])
            {
                continue;
            }
            for (uint32_t j = 0; j < pos.size(); ++j)
            {
                if (!dstComp[j])
                {
                    continue;
                }
                double d = CalculateDistance(pos[i], pos[j]);
                if (d < best)
                {
                    best = d;
                    info.gapMidpoint = Vector((pos[i].x + pos[j].x) / 2, (pos[i].y + pos[j].y) / 2, uav.z);
                }
            }
        }
        info.gapValid = std::isfinite(best);
        info.gapDistance = best;
    }
    return info;
}

} // namespace uavrsu
