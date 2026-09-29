#ifndef UAVRSU_ANIMATION_MANAGER_H
#define UAVRSU_ANIMATION_MANAGER_H

// EXPERIMENT LAYER - NetAnim visualisation of the recovery process.
// Vehicles (red, source/destination highlighted), RSUs (green = ACTIVE,
// black = FAILED) and the UAV (blue; orange while RECOVERING, green once
// RECOVERED) with descriptive labels, e.g. "RSU 2 FAILED", "UAV: RECOVERING
// step 4 East". Packet animation is skipped to keep the XML small; node
// movement is sampled every 0.25 s so UAV flights are visible.

#include "ns3/node-container.h"

#include <memory>
#include <string>

namespace ns3
{
class AnimationInterface;
}

namespace uavrsu
{

class RsuManager;

class AnimationManager
{
  public:
    AnimationManager();
    ~AnimationManager();

    void Enable(const std::string& path,
                const ns3::NodeContainer& vehicles,
                const RsuManager& rsus,
                ns3::Ptr<ns3::Node> uav,
                uint32_t source,
                uint32_t destination);
    bool IsEnabled() const;

    void OnRsuFailed(uint32_t rsuId, ns3::Ptr<ns3::Node> rsuNode);
    void SetUavStatus(const std::string& text, uint8_t r, uint8_t g, uint8_t b);

  private:
    std::unique_ptr<ns3::AnimationInterface> m_anim;
    ns3::Ptr<ns3::Node> m_uav;
};

} // namespace uavrsu

#endif
