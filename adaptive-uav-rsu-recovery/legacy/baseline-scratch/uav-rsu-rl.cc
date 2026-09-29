#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-module.h"
#include "ns3/applications-module.h" 
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/flow-monitor-module.h" 
#include "rsu-manager.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("UAVSUMOSimulation");

void
RepositionUAV(Ptr<Node> uavNode, Vector newPosition)
{
    Ptr<MobilityModel> mobility =
        uavNode->GetObject<MobilityModel>();

    mobility->SetPosition(newPosition);

    std::cout
        << "UAV REPOSITIONED to ("
        << newPosition.x << ", "
        << newPosition.y << ", "
        << newPosition.z
        << ") at "
        << Simulator::Now().GetSeconds()
        << " seconds\n";
}

void
MeasurePDR(Ptr<FlowMonitor> monitor, FlowMonitorHelper *flowmon)
{
    monitor->CheckForLostPackets();

    std::map<FlowId, FlowMonitor::FlowStats> stats =
        monitor->GetFlowStats();

    std::cout << "\n[PDR] Time = "
              << Simulator::Now().GetSeconds()
              << " s\n";

    for (auto const &flow : stats)
    {
        uint64_t tx = flow.second.txPackets;
        uint64_t rx = flow.second.rxPackets;

        double pdr = 0.0;

        if (tx > 0)
        {
            pdr = (static_cast<double>(rx) / tx) * 100.0;
        }

        std::cout << "Flow " << flow.first
                  << " | TX = " << tx
                  << " | RX = " << rx
                  << " | PDR = " << pdr << "%\n";
    }

    Simulator::Schedule(
        Seconds(1.0),
        &MeasurePDR,
        monitor,
        flowmon
    );
}

double//Modification
DistanceBetween(Ptr<Node> a, Ptr<Node> b)
{
    Ptr<MobilityModel> ma =
        a->GetObject<MobilityModel>();

    Ptr<MobilityModel> mb =
        b->GetObject<MobilityModel>();

    return ma->GetDistanceFrom(mb);
}


int main (int argc, char *argv[])
{
    std::string traceFile = "mobility.tcl";
    
    uint32_t numNodes = 20; 

    // 1. Create Nodes
    NodeContainer vehicleNodes;
    vehicleNodes.Create(numNodes);

    NodeContainer uavNode;
    uavNode.Create(1);

    RSUManager rsuManager; // modification
    rsuManager.Create(5); // modification
    rsuManager.InstallMobility(); // modification

    NodeContainer allNodes;
    allNodes.Add(vehicleNodes);
    allNodes.Add(uavNode);
    allNodes.Add(rsuManager.GetNodes()); // modification

    // 2. Mobility Setup
    Ns2MobilityHelper ns2 = Ns2MobilityHelper(traceFile);
    ns2.Install(vehicleNodes.Begin(), vehicleNodes.End());

    std::cout << "\n=========== VEHICLES ===========\n";

    for (uint32_t i = 0; i < vehicleNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> mobilityModel =
            vehicleNodes.Get(i)->GetObject<MobilityModel>();

        Vector position = mobilityModel->GetPosition();

        std::cout << "Vehicle " << i
                  << " | Position: ("
                  << position.x << ", "
                  << position.y << ", "
                  << position.z << ")"
                  << std::endl;
    } 

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(100.0, 100.0, 50.0));  
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(uavNode);

    std::cout << "\n=========== INITIAL DISTANCES ===========\n";

    std::cout << "Vehicle 0 -> RSU 2: "
            << DistanceBetween(vehicleNodes.Get(0),
                                rsuManager.GetNodes().Get(2))
            << " m\n";

    std::cout << "Vehicle 19 -> RSU 2: "
            << DistanceBetween(vehicleNodes.Get(19),
                                rsuManager.GetNodes().Get(2))
            << " m\n";

    std::cout << "Vehicle 0 -> Vehicle 19: "
            << DistanceBetween(vehicleNodes.Get(0),
                                vehicleNodes.Get(19))
            << " m\n";

    std::cout << "UAV -> RSU 2: "
            << DistanceBetween(uavNode.Get(0),
                                rsuManager.GetNodes().Get(2))
            << " m\n";

    // 3. Configure Wi-Fi
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211p); 

    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
    
    // --> ANTENNA POWER BOOST SO THEY CAN REACH THE DRONE
    wifiPhy.Set("TxPowerStart", DoubleValue(33.0)); // 33 dBm = strong signal
    wifiPhy.Set("TxPowerEnd", DoubleValue(33.0));

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer vehicleDevices = wifi.Install(wifiPhy, wifiMac, vehicleNodes);
    NetDeviceContainer uavDevice = wifi.Install(wifiPhy, wifiMac, uavNode);

    NetDeviceContainer rsuDevices = wifi.Install(wifiPhy, wifiMac, rsuManager.GetNodes()); // Modification

    // 4. Configure Internet Stack & AODV Routing
    AodvHelper aodv;
    Ipv4ListRoutingHelper list;
    list.Add(aodv, 10);

    InternetStackHelper internet;
    internet.SetRoutingHelper(list);
    
    // --> ACTIVE FOR PROPOSED SCENARIO: Internet installed on ALL nodes
    internet.Install(allNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer vehicleInterfaces = ipv4.Assign(vehicleDevices);
    
    // --> ACTIVE FOR PROPOSED SCENARIO: Gives the UAV an IP address!
    Ipv4InterfaceContainer uavInterface = ipv4.Assign(uavDevice);

    Ipv4InterfaceContainer rsuInterfaces = ipv4.Assign(rsuDevices); // modification.

    // ---------------------------------------------------------
    // 5. GENERATE REAL DATA TRAFFIC (THE PAYLOAD)
    // ---------------------------------------------------------
    uint16_t port = 9;
    
    // The "Receiver" car
    UdpEchoServerHelper echoServer(port);
    ApplicationContainer serverApps = echoServer.Install(vehicleNodes.Get(numNodes - 1));
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(30.0));

    // The "Sender" car sending 1000 packets
    Ipv4Address serverAddress = vehicleInterfaces.GetAddress(numNodes - 1);
    UdpEchoClientHelper echoClient(serverAddress, port);
    echoClient.SetAttribute("MaxPackets", UintegerValue(1000));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(0.05))); 
    echoClient.SetAttribute("PacketSize", UintegerValue(1024)); 

    ApplicationContainer clientApps = echoClient.Install(vehicleNodes.Get(0));
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(30.0));
    // ---------------------------------------------------------

    AnimationInterface anim("uav-rsu-rl-animation.xml"); // Modification
    // anim.SetConstantPosition(uavNode.Get(0), 100, 100);
    anim.UpdateNodeColor(uavNode.Get(0), 0, 0, 255);

    // Vehicles - Red // mod
    for(uint32_t i=0;i<vehicleNodes.GetN();i++)
    {
        anim.UpdateNodeColor(vehicleNodes.Get(i),255,0,0);
    }

    // UAV - Blue
    anim.UpdateNodeColor(uavNode.Get(0),0,0,255);

    // RSUs - Green
    for(uint32_t i=0;i<rsuManager.GetNodes().GetN();i++)
    {
        anim.UpdateNodeColor(rsuManager.GetNodes().Get(i),0,255,0);
    }  //mod

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Schedule(
    Seconds(5.0),
    &MeasurePDR,
    monitor,
    &flowmon
    );

    Simulator::Schedule(
    Seconds(10.0),
    &RSUManager::FailRSU,
    &rsuManager,
    2
    );

    Simulator::Schedule(
    Seconds(12.0),
    &RepositionUAV,
    uavNode.Get(0),
    Vector(300.0, 100.0, 50.0)
    );

    Simulator::Stop(Seconds(30.0));

    std::cout << "\n=========== RSUs ===========\n";

    for (auto &rsu : rsuManager.GetRSUs()) // mod
    {
        std::cout
            << "RSU "
            << rsu.id
            << " | Position: ("
            << rsu.position.x << ", "
            << rsu.position.y << ", "
            << rsu.position.z
            << ") | Status: "
            << (rsu.active ? "ACTIVE" : "FAILED")
            << "\n";
    } //mod

    Simulator::Run();

    // Outputs the raw data to XML
    // monitor->SerializeToXmlFile("vanet_results.xml", true, true);
    monitor->SerializeToXmlFile("milestone1_rsu_results.xml", true, true);
    Simulator::Destroy();

    return 0;
}