#include "animation-manager.h"

#include "../env/rsu-manager.h"

#include "ns3/netanim-module.h"

namespace uavrsu
{

using namespace ns3;

AnimationManager::AnimationManager() = default;
AnimationManager::~AnimationManager() = default;

void
AnimationManager::Enable(const std::string& path,
                         const NodeContainer& vehicles,
                         const RsuManager& rsus,
                         Ptr<Node> uav,
                         uint32_t source,
                         uint32_t destination)
{
    m_anim = std::make_unique<AnimationInterface>(path);
    m_anim->SkipPacketTracing();
    m_anim->SetMobilityPollInterval(Seconds(0.25));
    m_uav = uav;

    for (uint32_t i = 0; i < vehicles.GetN(); ++i)
    {
        std::string label = "Vehicle " + std::to_string(i);
        if (i == source)
        {
            label += " (SRC)";
        }
        if (i == destination)
        {
            label += " (DST)";
        }
        m_anim->UpdateNodeDescription(vehicles.Get(i), label);
        m_anim->UpdateNodeColor(vehicles.Get(i), 255, 0, 0);
        m_anim->UpdateNodeSize(vehicles.Get(i)->GetId(), 8, 8);
    }
    for (const auto& r : rsus.GetRsus())
    {
        m_anim->UpdateNodeDescription(r.node, "RSU " + std::to_string(r.id) + " ACTIVE");
        m_anim->UpdateNodeColor(r.node, 0, 200, 0);
        m_anim->UpdateNodeSize(r.node->GetId(), 14, 14);
    }
    m_anim->UpdateNodeSize(uav->GetId(), 16, 16);
    SetUavStatus("UAV: IDLE", 0, 0, 255);
}

bool
AnimationManager::IsEnabled() const
{
    return m_anim != nullptr;
}

void
AnimationManager::OnRsuFailed(uint32_t rsuId, Ptr<Node> rsuNode)
{
    if (!m_anim)
    {
        return;
    }
    m_anim->UpdateNodeDescription(rsuNode, "RSU " + std::to_string(rsuId) + " FAILED");
    m_anim->UpdateNodeColor(rsuNode, 0, 0, 0);
}

void
AnimationManager::SetUavStatus(const std::string& text, uint8_t r, uint8_t g, uint8_t b)
{
    if (!m_anim)
    {
        return;
    }
    m_anim->UpdateNodeDescription(m_uav, text);
    m_anim->UpdateNodeColor(m_uav, r, g, b);
}

} // namespace uavrsu
