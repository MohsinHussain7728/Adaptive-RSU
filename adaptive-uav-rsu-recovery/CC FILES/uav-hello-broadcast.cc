#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("UAVHelloBroadcast");

// Vehicle receiving function
void ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Packet> packet;

    while ((packet = socket->Recv()))
    {
        NS_LOG_UNCOND("Vehicle received HELLO packet at "
        << Simulator::Now().GetSeconds() << " seconds");
    }
}

// UAV broadcast function
void SendHello(Ptr<Socket> socket)
{
    Ptr<Packet> packet = Create<Packet>(100);

    socket->Send(packet);

    NS_LOG_UNCOND("UAV broadcast HELLO at "
    << Simulator::Now().GetSeconds() << " seconds");

    Simulator::Schedule(Seconds(1.0), &SendHello, socket);
}

int main(int argc, char *argv[])
{
    uint32_t numVehicles = 20;

    NodeContainer vehicles;
    vehicles.Create(numVehicles);

    NodeContainer uav;
    uav.Create(1);

    NodeContainer allNodes;
    allNodes.Add(vehicles);
    allNodes.Add(uav);

    // Install internet stack
    InternetStackHelper internet;
    internet.Install(allNodes);

    // Import SUMO mobility
    Ns2MobilityHelper ns2("mobility.tcl");
    ns2.Install();

    // UAV mobility (50m altitude)
    MobilityHelper mobility;

    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(100.0, 100.0, 50.0));

    mobility.SetPositionAllocator(pos);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    mobility.Install(uav);

    YansWifiPhyHelper phy;
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    phy.SetChannel(channel.Create());

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211p);

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install(phy, mac, allNodes);

    // Assign IP addresses
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");

    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

    // Create receiving sockets for vehicles
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

    for (uint32_t i = 0; i < numVehicles; i++)
    {
        Ptr<Socket> recvSink = Socket::CreateSocket(vehicles.Get(i), tid);

        InetSocketAddress local =
        InetSocketAddress(Ipv4Address::GetAny(), 80);

        recvSink->Bind(local);
        recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));
    }

    // UAV broadcast socket
    Ptr<Socket> source = Socket::CreateSocket(uav.Get(0), tid);

    InetSocketAddress broadcastAddr =
    InetSocketAddress(Ipv4Address("255.255.255.255"), 80);

    source->SetAllowBroadcast(true);
    source->Connect(broadcastAddr);

    Simulator::Schedule(Seconds(1.0), &SendHello, source);

    // NetAnim visualization
    AnimationInterface anim("uav-hello.xml");

    anim.SetConstantPosition(uav.Get(0), 100, 100);

    Simulator::Stop(Seconds(30.0));

    Simulator::Run();
    Simulator::Destroy();

    return 0;
}