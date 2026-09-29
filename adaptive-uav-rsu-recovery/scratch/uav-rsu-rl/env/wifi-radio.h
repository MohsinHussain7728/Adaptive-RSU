#ifndef UAV_RSU_WIFI_RADIO_H
#define UAV_RSU_WIFI_RADIO_H

#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/wifi-phy.h"

namespace uavrsu
{

/**
 * Switches a Wi-Fi PHY off as soon as it is not transmitting.
 *
 * ns-3.47 FrameExchangeManager::SendMpdu schedules TransmissionSucceeded() for
 * frames without acknowledgment (broadcasts such as AODV HELLO/RREQ) without
 * keeping the event id. WifiPhy::SetOffMode() resets the MAC (m_dcf = nullptr)
 * but that event still fires and dereferences the null channel access function
 * (segfault). Deferring the switch-off to the end of the ongoing transmission
 * (at most one frame, ~1.5 ms) avoids that.
 */
inline void
SwitchOffPhyWhenIdle(ns3::Ptr<ns3::WifiPhy> phy)
{
    if (!phy || phy->IsStateOff())
    {
        return;
    }
    if (phy->IsStateTx())
    {
        ns3::Simulator::Schedule(phy->GetDelayUntilIdle() + ns3::MicroSeconds(1),
                                 &SwitchOffPhyWhenIdle,
                                 phy);
        return;
    }
    phy->SetOffMode();
}

} // namespace uavrsu

#endif // UAV_RSU_WIFI_RADIO_H
