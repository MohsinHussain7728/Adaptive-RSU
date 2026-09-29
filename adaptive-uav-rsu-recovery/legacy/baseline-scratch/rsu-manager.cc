#include "rsu-manager.h"
#include "ns3/wifi-net-device.h"

RSUManager::RSUManager()
{
}

void
RSUManager::Create(uint32_t numberOfRsus)
{
    rsuNodes.Create(numberOfRsus);

    std::vector<Vector> positions =
    {
        Vector(100,100,0),

        Vector(100,300,0),

        Vector(300,100,0),

        Vector(300,300,0),

        Vector(200,200,0)
    };

    rsus.clear();

    for(uint32_t i=0;i<numberOfRsus;i++)
    {
        RSUInfo info;

        info.id=i;

        info.node=rsuNodes.Get(i);

        info.position=positions[i];

        info.active=true;

        rsus.push_back(info);
    }
}

void
RSUManager::InstallMobility()
{
    MobilityHelper mobility;

    Ptr<ListPositionAllocator> allocator=
    CreateObject<ListPositionAllocator>();

    for(auto &r:rsus)
    {
        allocator->Add(r.position);
    }

    mobility.SetPositionAllocator(allocator);

    mobility.SetMobilityModel(
        "ns3::ConstantPositionMobilityModel"
    );

    mobility.Install(rsuNodes);
}

NodeContainer
RSUManager::GetNodes()
{
    return rsuNodes;
}

std::vector<RSUInfo>&
RSUManager::GetRSUs()
{
    return rsus;
}

void
RSUManager::FailRSU(uint32_t rsuId)
{
    if (rsuId >= rsus.size())
    {
        return;
    }

    if (!rsus[rsuId].active)
    {
        return;
    }

    rsus[rsuId].active = false;

    Ptr<Node> rsuNode = rsus[rsuId].node;

    for (uint32_t i = 0; i < rsuNode->GetNDevices(); ++i)
    {
        Ptr<WifiNetDevice> wifiDevice =
            DynamicCast<WifiNetDevice>(rsuNode->GetDevice(i));

        if (wifiDevice)
        {
            wifiDevice->GetPhy()->SetOffMode();
        }
    }

    std::cout << "RSU " << rsuId
              << " FAILED at "
              << Simulator::Now().GetSeconds()
              << " seconds\n";
}
