#include "rsu-manager.h"

#include "wifi-radio.h"

#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/simulator.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy.h"

#include <iostream>

NS_LOG_COMPONENT_DEFINE("UavRsuRsuManager");

namespace uavrsu
{

using namespace ns3;

void
RsuManager::Create(const std::vector<Vector>& positions)
{
    m_nodes.Create(positions.size());
    m_rsus.clear();
    for (uint32_t i = 0; i < positions.size(); ++i)
    {
        RsuInfo info;
        info.id = i;
        info.node = m_nodes.Get(i);
        info.position = positions[i];
        m_rsus.push_back(info);
    }
}

void
RsuManager::InstallMobility()
{
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> alloc = CreateObject<ListPositionAllocator>();
    for (const auto& r : m_rsus)
    {
        alloc->Add(r.position);
    }
    mobility.SetPositionAllocator(alloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(m_nodes);
}

NodeContainer
RsuManager::GetNodes() const
{
    return m_nodes;
}

std::vector<RsuInfo>&
RsuManager::GetRsus()
{
    return m_rsus;
}

const std::vector<RsuInfo>&
RsuManager::GetRsus() const
{
    return m_rsus;
}

uint32_t
RsuManager::GetN() const
{
    return m_rsus.size();
}

bool
RsuManager::IsActive(uint32_t id) const
{
    return id < m_rsus.size() && m_rsus[id].active;
}

void
RsuManager::FailRsu(uint32_t id)
{
    if (id >= m_rsus.size() || !m_rsus[id].active)
    {
        return;
    }
    m_rsus[id].active = false;
    m_rsus[id].failureTime = Simulator::Now().GetSeconds();

    Ptr<Node> node = m_rsus[id].node;
    for (uint32_t i = 0; i < node->GetNDevices(); ++i)
    {
        Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(node->GetDevice(i));
        if (dev)
        {
            SwitchOffPhyWhenIdle(dev->GetPhy());
        }
    }
    NS_LOG_INFO("RSU " << id << " FAILED at " << Simulator::Now().GetSeconds() << " s");
    for (auto& cb : m_listeners)
    {
        cb(id);
    }
}

void
RsuManager::AddFailureListener(Callback<void, uint32_t> cb)
{
    m_listeners.push_back(cb);
}

void
RsuManager::Print(std::ostream& os) const
{
    for (const auto& r : m_rsus)
    {
        os << "RSU " << r.id << " | node " << r.node->GetId() << " | (" << r.position.x << ", "
           << r.position.y << ", " << r.position.z << ") | " << (r.active ? "ACTIVE" : "FAILED")
           << "\n";
    }
}

} // namespace uavrsu
