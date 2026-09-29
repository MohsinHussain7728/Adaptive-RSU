#ifndef RSU_MANAGER_H
#define RSU_MANAGER_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"

using namespace ns3;

struct RSUInfo
{
    uint32_t id;

    Ptr<Node> node;

    Ptr<NetDevice> device;

    Vector position;

    bool active;
};

class RSUManager
{
public:

    RSUManager();

    void Create(uint32_t numberOfRsus);

    void InstallMobility();

    NodeContainer GetNodes();

    std::vector<RSUInfo>& GetRSUs();

    void FailRSU(uint32_t rsuId);



private:

    NodeContainer rsuNodes;

    std::vector<RSUInfo> rsus;
};

#endif
