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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("UAVSUMOSimulation");

int main (int argc, char *argv[])
{
    std::string traceFile = "mobility.tcl";
    
    uint32_t numNodes = 20; 

    // 1. Create Nodes
    NodeContainer vehicleNodes;
    vehicleNodes.Create(numNodes);

    NodeContainer uavNode;
    uavNode.Create(1);

    NodeContainer allNodes;
    allNodes.Add(vehicleNodes);
    allNodes.Add(uavNode);

    // 2. Mobility Setup
    Ns2MobilityHelper ns2 = Ns2MobilityHelper(traceFile);
    ns2.Install(vehicleNodes.Begin(), vehicleNodes.End()); 

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(100.0, 100.0, 50.0));  
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(uavNode);

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

    AnimationInterface anim("uav-sumo-animation.xml");
    anim.SetConstantPosition(uavNode.Get(0), 100, 100);

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(30.0));
    Simulator::Run();

    // Outputs the raw data to XML
    monitor->SerializeToXmlFile("vanet_results.xml", true, true);
    Simulator::Destroy();

    return 0;
}