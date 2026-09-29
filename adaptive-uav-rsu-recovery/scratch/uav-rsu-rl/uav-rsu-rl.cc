// ============================================================================
// Adaptive UAV-Assisted VANET Recovery using Reinforcement Learning
//
// Entry point. Builds one or more independent ns-3 simulation runs (episodes)
// of an urban IEEE 802.11p VANET (SUMO or static mobility, AODV, UDP echo
// Vehicle src -> Vehicle dst), fails an RSU, and lets a recovery strategy move
// a UAV relay with relative movement actions until communication is restored.
//
// Code organisation (see README):
//   rl/          GENERIC RL LAYER - RLAgent interface, QLearningAgent, factory.
//                No ns-3 code; replaceable algorithms.
//   env/         ENVIRONMENT LAYER - RSU failure, UAV movement, network
//                monitoring, topology observation, state encoding, reward,
//                recovery strategies, the recovery loop.
//   experiment/  EXPERIMENT LAYER - CSV writers, NetAnim.
//   this file    Scenario/mobility/radio/stack/traffic construction and the
//                training / evaluation episode loop.
//
// The simulation talks to the RL algorithm only through uavrl::RLAgent. To use
// another algorithm, implement RLAgent, register it in rl/rl-agent-factory.cc
// and run with --rlAlgorithm=<name>.
// ============================================================================

#include "env/network-monitor.h"
#include "env/recovery-environment.h"
#include "env/recovery-strategy.h"
#include "env/reward-function.h"
#include "env/rsu-manager.h"
#include "env/scenario.h"
#include "env/sim-config.h"
#include "env/state-encoder.h"
#include "env/topology-observer.h"
#include "env/uav-controller.h"
#include "experiment/animation-manager.h"
#include "experiment/results-writer.h"
#include "rl/rl-agent-factory.h"

#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/arp-cache.h"
#include "ns3/ipv4-interface.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/wifi-module.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace ns3;
using namespace uavrsu;

NS_LOG_COMPONENT_DEFINE("UavRsuRl");

namespace
{

std::vector<std::string>
SplitList(const std::string& s)
{
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        if (!item.empty())
        {
            out.push_back(item);
        }
    }
    return out;
}

bool
FileExists(const std::string& path)
{
    std::ifstream f(path);
    return static_cast<bool>(f);
}

OperationalArea
ResolveArea(const SimConfig& cfg, const ScenarioGeometry& geo)
{
    OperationalArea a;
    a.minX = std::isnan(cfg.areaMinX) ? geo.WestMinX() - 100.0 : cfg.areaMinX;
    a.maxX = std::isnan(cfg.areaMaxX) ? geo.EastMaxX() + 100.0 : cfg.areaMaxX;
    a.minY = std::isnan(cfg.areaMinY) ? -100.0 : cfg.areaMinY;
    a.maxY = std::isnan(cfg.areaMaxY) ? geo.Height() + 100.0 : cfg.areaMaxY;
    return a;
}

// ns-3 ARP marks a neighbour DEAD for 100 s after 3 lost requests, which on a
// busy shared ad hoc channel silently black-holes a node for most of a run.
// The entries must be PERMANENT (not NeighborCacheHelper's auto-generated
// state): AODV maps MAC transmit failures to neighbours only through ALIVE or
// PERMANENT ARP entries, and without that feedback a failed next hop is never
// declared broken while traffic keeps refreshing it.
void
PopulatePermanentArp(const NodeContainer& nodes)
{
    struct Iface
    {
        Ipv4Address ip;
        Address mac;
    };
    std::vector<Iface> ifaces;
    for (uint32_t n = 0; n < nodes.GetN(); ++n)
    {
        Ptr<Ipv4L3Protocol> ip = nodes.Get(n)->GetObject<Ipv4L3Protocol>();
        for (uint32_t i = 0; i < ip->GetNInterfaces(); ++i)
        {
            if (ip->GetInterface(i)->GetArpCache())
            {
                ifaces.push_back({ip->GetAddress(i, 0).GetLocal(), ip->GetInterface(i)->GetDevice()->GetAddress()});
            }
        }
    }
    for (uint32_t n = 0; n < nodes.GetN(); ++n)
    {
        Ptr<Ipv4L3Protocol> ip = nodes.Get(n)->GetObject<Ipv4L3Protocol>();
        for (uint32_t i = 0; i < ip->GetNInterfaces(); ++i)
        {
            Ptr<ArpCache> cache = ip->GetInterface(i)->GetArpCache();
            if (!cache)
            {
                continue;
            }
            for (const auto& peer : ifaces)
            {
                if (peer.ip == ip->GetAddress(i, 0).GetLocal())
                {
                    continue;
                }
                ArpCache::Entry* e = cache->Lookup(peer.ip);
                if (!e)
                {
                    e = cache->Add(peer.ip);
                }
                e->SetMacAddress(peer.mac);
                e->MarkPermanent();
            }
        }
    }
}

struct ProcessContext
{
    SimConfig cfg;
    ScenarioInfo scenario;
    uint32_t numVehicles = 0;
    std::vector<std::string> variants;
    OperationalArea area;
    std::vector<Vector> rsuPositions;
};

// ----------------------------------------------------------------------------
// One episode = one complete, independent ns-3 simulation run.
// ----------------------------------------------------------------------------
void
RunEpisode(const ProcessContext& pc,
           uint32_t episodeInProcess,
           bool lastEpisode,
           uavrl::RLAgent* agent,
           StateEncoder& encoder,
           uint64_t& globalEpisodeCounter)
{
    const SimConfig& cfg = pc.cfg;
    const uint32_t runNumber = cfg.run + episodeInProcess;
    RngSeedManager::SetSeed(cfg.seed);
    RngSeedManager::SetRun(runNumber);
    RngSeedManager::ResetNextStreamIndex();
    Ipv4AddressGenerator::Reset();

    const uint32_t n = pc.numVehicles;
    const uint32_t src = cfg.sourceVehicle;
    const uint32_t dst = cfg.destinationVehicle < 0 ? n - 1 : static_cast<uint32_t>(cfg.destinationVehicle);
    const std::string variant = pc.variants[episodeInProcess % pc.variants.size()];

    // ---------------- nodes ----------------
    NodeContainer vehicles;
    vehicles.Create(n);
    NodeContainer uavNodes;
    uavNodes.Create(1);
    RsuManager rsus;
    rsus.Create(pc.rsuPositions);
    rsus.InstallMobility();

    NodeContainer all;
    all.Add(vehicles);
    all.Add(uavNodes);
    all.Add(rsus.GetNodes());

    // ---------------- MOBILITY LAYER ----------------
    std::string traceFile;
    if (cfg.mobility == "sumo")
    {
        traceFile = !cfg.traceFile.empty() ? cfg.traceFile
                                           : pc.scenario.directory + "/mobility_" + variant + ".tcl";
        if (!FileExists(traceFile))
        {
            NS_FATAL_ERROR("Mobility trace not found: " << traceFile
                                                        << " (run scratch/sumo-scenario/generate_all.sh)");
        }
        Ns2MobilityHelper ns2(traceFile);
        ns2.Install(vehicles.Begin(), vehicles.End());
    }
    else
    {
        MobilityHelper mob;
        Ptr<ListPositionAllocator> alloc = CreateObject<ListPositionAllocator>();
        for (const auto& p : StaticVehicleLayout(pc.scenario.geometry, n))
        {
            alloc->Add(p);
        }
        mob.SetPositionAllocator(alloc);
        mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mob.Install(vehicles);
    }

    // UAV start position (configurable, optionally random for training diversity).
    Vector home = DefaultUavHome(pc.scenario.geometry, cfg.uavZ);
    Vector start(std::isnan(cfg.uavX) ? home.x : cfg.uavX, std::isnan(cfg.uavY) ? home.y : cfg.uavY, cfg.uavZ);
    if (cfg.randomUavStart)
    {
        Ptr<UniformRandomVariable> u = CreateObject<UniformRandomVariable>();
        start.x = u->GetValue(pc.area.minX, pc.area.maxX);
        start.y = u->GetValue(pc.area.minY, pc.area.maxY);
    }
    UavController uav;
    uav.Install(uavNodes.Get(0), start, pc.area, cfg.movementStep, cfg.uavSpeed, cfg.numActions);

    // ---------------- IEEE 802.11p radio (single ad hoc channel) ----------------
    // Log-distance loss (5.9 GHz reference loss) inside a hard communication
    // range; beyond commRange nothing is received (no interference either).
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211p);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue(cfg.phyMode),
                                 "ControlMode", StringValue(cfg.phyMode),
                                 "NonUnicastMode", StringValue(cfg.phyMode));
    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                               "Exponent", DoubleValue(cfg.pathLossExponent),
                               "ReferenceLoss", DoubleValue(47.86));
    channel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(cfg.commRange));
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("TxPowerStart", DoubleValue(cfg.txPowerDbm));
    phy.Set("TxPowerEnd", DoubleValue(cfg.txPowerDbm));
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer vehicleDevs = wifi.Install(phy, mac, vehicles);
    NetDeviceContainer uavDevs = wifi.Install(phy, mac, uavNodes);
    NetDeviceContainer rsuDevs = wifi.Install(phy, mac, rsus.GetNodes());

    // ---------------- AODV + IPv4 ----------------
    AodvHelper aodv;
    aodv.Set("HelloInterval", TimeValue(Seconds(cfg.aodvHelloInterval)));
    aodv.Set("AllowedHelloLoss", UintegerValue(cfg.aodvAllowedHelloLoss));
    aodv.Set("RreqRetries", UintegerValue(cfg.aodvRreqRetries));
    aodv.Set("NetTraversalTime", TimeValue(Seconds(cfg.aodvNetTraversalTime)));
    aodv.Set("PathDiscoveryTime", TimeValue(Seconds(2 * cfg.aodvNetTraversalTime)));
    aodv.Set("BlackListTimeout", TimeValue(Seconds(cfg.aodvRreqRetries * cfg.aodvNetTraversalTime)));
    aodv.Set("ActiveRouteTimeout", TimeValue(Seconds(cfg.aodvActiveRouteTimeout)));
    aodv.Set("MaxQueueTime", TimeValue(Seconds(cfg.aodvMaxQueueTime)));
    Ipv4ListRoutingHelper list;
    list.Add(aodv, 10);
    InternetStackHelper internet;
    internet.SetRoutingHelper(list);
    internet.Install(all);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer vehicleIfs = ipv4.Assign(vehicleDevs);
    Ipv4InterfaceContainer uavIfs = ipv4.Assign(uavDevs);
    Ipv4InterfaceContainer rsuIfs = ipv4.Assign(rsuDevs);

    if (cfg.staticArp)
    {
        PopulatePermanentArp(all);
    }

    std::map<Ipv4Address, std::string> labels;
    for (uint32_t i = 0; i < n; ++i)
    {
        labels[vehicleIfs.GetAddress(i)] = "V" + std::to_string(i);
    }
    labels[uavIfs.GetAddress(0)] = "UAV";
    for (uint32_t i = 0; i < rsus.GetN(); ++i)
    {
        labels[rsuIfs.GetAddress(i)] = "RSU" + std::to_string(i);
    }

    // ---------------- application traffic ----------------
    const uint16_t port = 9;
    UdpEchoServerHelper server(port);
    ApplicationContainer serverApp = server.Install(vehicles.Get(dst));
    serverApp.Start(Seconds(1.0));
    serverApp.Stop(Seconds(cfg.simulationTime));

    UdpEchoClientHelper client(vehicleIfs.GetAddress(dst), port);
    uint32_t maxPackets = static_cast<uint32_t>((cfg.simulationTime - cfg.trafficStart) / cfg.packetInterval) + 10;
    client.SetAttribute("MaxPackets", UintegerValue(maxPackets));
    client.SetAttribute("Interval", TimeValue(Seconds(cfg.packetInterval)));
    client.SetAttribute("PacketSize", UintegerValue(cfg.packetSize));
    ApplicationContainer clientApp = client.Install(vehicles.Get(src));
    clientApp.Start(Seconds(cfg.trafficStart));
    clientApp.Stop(Seconds(cfg.simulationTime));

    if (cfg.backgroundFlows > 0 && n > 3)
    {
        Ptr<UniformRandomVariable> pick = CreateObject<UniformRandomVariable>();
        for (uint32_t f = 0; f < cfg.backgroundFlows; ++f)
        {
            uint32_t a = pick->GetInteger(0, n - 1);
            uint32_t b = pick->GetInteger(0, n - 1);
            if (a == b)
            {
                b = (b + 1) % n;
            }
            uint16_t bgPort = 4000 + f;
            UdpServerHelper bgServer(bgPort);
            ApplicationContainer s = bgServer.Install(vehicles.Get(b));
            s.Start(Seconds(1.0));
            UdpClientHelper bgClient(vehicleIfs.GetAddress(b), bgPort);
            bgClient.SetAttribute("MaxPackets", UintegerValue(1000000));
            bgClient.SetAttribute("Interval", TimeValue(Seconds(1.0 / cfg.backgroundRate)));
            bgClient.SetAttribute("PacketSize", UintegerValue(512));
            ApplicationContainer c = bgClient.Install(vehicles.Get(a));
            c.Start(Seconds(cfg.trafficStart + pick->GetValue(0, 1)));
            c.Stop(Seconds(cfg.simulationTime));
        }
    }

    // ---------------- environment components ----------------
    NetworkMonitor monitor;
    monitor.Configure(cfg.packetSize, cfg.deliveryGrace);
    monitor.AttachApplications(clientApp.Get(0), serverApp.Get(0));
    monitor.AttachNodes(all);

    TopologyObserver topology;
    topology.Configure(cfg.commRange * cfg.linkMargin);
    topology.SetNodes(vehicles, &rsus, uavNodes.Get(0), src, dst);

    double nominalKbps = cfg.packetSize * 8.0 / cfg.packetInterval / 1000.0;
    RewardFunction reward(cfg.reward, cfg.movementStep, cfg.commRange * cfg.linkMargin, nominalKbps, cfg.gamma);
    std::unique_ptr<RecoveryStrategy> strategy = CreateRecoveryStrategy(cfg.strategy, agent);

    AnimationManager animation;
    if (cfg.netanim && lastEpisode)
    {
        animation.Enable(cfg.outputDir + "/" + cfg.animFile, vehicles, rsus, uavNodes.Get(0), src, dst);
    }

    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> flowMonitor = flowHelper.InstallAll();

    RunContext rc;
    rc.label = cfg.runLabel;
    rc.mode = cfg.mode;
    rc.strategy = cfg.strategy;
    rc.algorithm = cfg.UsesRl() ? cfg.rlAlgorithm : "-";
    rc.scenario = cfg.mobility == "sumo" ? pc.scenario.name : "static";
    rc.traceVariant = cfg.mobility == "sumo" ? variant : "-";
    rc.numVehicles = n;
    rc.seed = cfg.seed;
    rc.run = runNumber;
    rc.failedRsu = cfg.failedRsu;
    rc.episode = static_cast<uint32_t>(globalEpisodeCounter);
    {
        std::ostringstream id;
        id << cfg.mode << "_" << cfg.strategy;
        if (cfg.UsesRl())
        {
            id << "-" << cfg.rlAlgorithm;
        }
        id << "_" << rc.scenario << "_v" << rc.traceVariant << "_f" << cfg.failedRsu << "_r" << runNumber;
        rc.runId = id.str();
    }

    EnvironmentComponents comp;
    comp.config = &cfg;
    comp.rsus = &rsus;
    comp.uav = &uav;
    comp.monitor = &monitor;
    comp.topology = &topology;
    comp.encoder = &encoder;
    comp.reward = &reward;
    comp.strategy = strategy.get();
    comp.animation = animation.IsEnabled() ? &animation : nullptr;
    comp.sourceNode = vehicles.Get(src);
    comp.destinationAddress = vehicleIfs.GetAddress(dst);
    comp.addressLabels = labels;
    comp.failedRsuNodeId = cfg.failedRsu >= 0 ? static_cast<int32_t>(rsus.GetRsus()[cfg.failedRsu].node->GetId()) : -1;
    comp.run = rc;
    comp.globalEpisodeCounter = &globalEpisodeCounter;

    RecoveryEnvironment env(comp);
    env.Start();

    if (cfg.verbose)
    {
        std::cout << "\n=== RUN " << rc.runId << " | vehicles=" << n << " src=V" << src << " dst=V" << dst
                  << " | UAV start=(" << start.x << "," << start.y << "," << start.z << ")"
                  << (traceFile.empty() ? "" : " | trace=" + traceFile) << "\n";
        rsus.Print(std::cout);
    }

    Simulator::Stop(Seconds(cfg.simulationTime));
    Simulator::Run();
    double simEnd = Simulator::Now().GetSeconds();

    env.Finish(simEnd);
    if (cfg.flowmonXml && lastEpisode)
    {
        flowMonitor->CheckForLostPackets();
        flowMonitor->SerializeToXmlFile(cfg.outputDir + "/flowmon_results.xml", true, true);
    }
    if (cfg.verbose)
    {
        std::cout << "=== END " << rc.runId << " at t=" << simEnd << " s | monitored flow tx="
                  << monitor.GetTotalTx() << " rx=" << monitor.GetTotalRx() << " | UAV distance="
                  << uav.GetTotalDistance() << " m\n";
    }
    if (cfg.diagnostics)
    {
        monitor.PrintDiagnostics(std::cout);
    }
    Simulator::Destroy();
}

} // namespace

int
main(int argc, char* argv[])
{
    ProcessContext pc;
    SimConfig& cfg = pc.cfg;
    CommandLine cmd(__FILE__);
    cfg.Register(cmd);
    cmd.Parse(argc, argv);

    std::string err = cfg.Validate();
    if (!err.empty())
    {
        std::cerr << "Invalid configuration: " << err << "\n";
        return 1;
    }
    EnsureDirectory(cfg.outputDir);
    EnsureDirectory(cfg.outputDir + "/routing_tables");

    // ---------------- scenario ----------------
    if (cfg.mobility == "sumo")
    {
        pc.scenario = LoadScenarioInfo(cfg.sumoScenario + "/" + cfg.vehicleDensity);
        if (!pc.scenario.loadedFromFile)
        {
            std::cerr << "SUMO scenario not found in " << cfg.sumoScenario << "/" << cfg.vehicleDensity
                      << " (generate it with scratch/sumo-scenario/generate_all.sh)\n";
            return 1;
        }
        if (cfg.numVehicles != 0 && cfg.numVehicles != pc.scenario.numVehicles)
        {
            std::cerr << "--numVehicles=" << cfg.numVehicles << " but scenario '" << pc.scenario.name << "' has "
                      << pc.scenario.numVehicles << " vehicles; regenerate it with generate_scenario.py --vehicles "
                      << cfg.numVehicles << "\n";
            return 1;
        }
        pc.numVehicles = pc.scenario.numVehicles;
    }
    else
    {
        pc.numVehicles = cfg.numVehicles ? cfg.numVehicles : 20;
    }
    if (cfg.sourceVehicle >= pc.numVehicles || cfg.destinationVehicle >= static_cast<int32_t>(pc.numVehicles))
    {
        std::cerr << "source/destination vehicle index out of range\n";
        return 1;
    }
    pc.variants = SplitList(cfg.traceVariants);
    if (pc.variants.empty())
    {
        pc.variants = {"1"};
    }
    pc.area = ResolveArea(cfg, pc.scenario.geometry);
    pc.rsuPositions = ComputeRsuLayout(pc.scenario.geometry, cfg.numRsus);

    // ---------------- STATE ENCODER (environment side) ----------------
    std::unique_ptr<StateEncoder> encoder =
        CreateStateEncoder(cfg.stateEncoder, cfg.commRange * cfg.linkMargin, cfg.pdrThreshold, cfg.stateDistanceBins,
                           pc.area.maxX - pc.area.minX, pc.area.maxY - pc.area.minY);
    if (!encoder)
    {
        std::cerr << "Unknown --stateEncoder=" << cfg.stateEncoder << "\n";
        return 1;
    }
    {
        std::string path = cfg.outputDir + "/state_encoding.csv";
        if (!FileExists(path))
        {
            std::ofstream out(path);
            out << "state_id,description\n";
            for (uint32_t s = 0; s < encoder->GetNumStates(); ++s)
            {
                out << s << "," << encoder->Describe(s) << "\n";
            }
        }
    }

    // ============================================================
    // GENERIC RL AGENT CREATION
    // Only the algorithm NAME and a generic parameter map are handed over;
    // the simulation never sees algorithm-specific data structures.
    // ============================================================
    std::unique_ptr<uavrl::RLAgent> agent;
    if (cfg.UsesRl())
    {
        uavrl::AgentConfig ac;
        ac.numStates = encoder->GetNumStates();
        ac.numFeatures = encoder->GetNumFeatures();
        ac.numActions = cfg.numActions;
        ac.seed = static_cast<uint64_t>(cfg.seed) * 1000003ULL + cfg.run;
        ac.params = {{"alpha", cfg.alpha},
                     {"gamma", cfg.gamma},
                     {"epsilon", cfg.epsilon},
                     {"epsilonMin", cfg.epsilonMin},
                     {"epsilonDecay", cfg.epsilonDecay},
                     {"evalEpsilon", cfg.evalEpsilon},
                     {"initQ", cfg.initQ},
                     {"episodeIndex", static_cast<double>(cfg.episodeIndex)}};
        agent = uavrl::CreateRLAgent(cfg.rlAlgorithm, ac);
        if (!agent)
        {
            std::cerr << "Unknown --rlAlgorithm=" << cfg.rlAlgorithm << ". Available:";
            for (const auto& a : uavrl::AvailableRLAlgorithms())
            {
                std::cerr << " " << a;
            }
            std::cerr << "\n";
            return 1;
        }
        agent->SetTrainingMode(cfg.IsTraining());
        const std::string model = cfg.ModelPath();
        if (FileExists(model))
        {
            if (!agent->LoadModel(model))
            {
                std::cerr << "Model " << model << " is incompatible with the current state/action space\n";
                return 1;
            }
            std::cout << "Loaded model " << model << "\n";
        }
        else if (!cfg.IsTraining() && cfg.rlAlgorithm != "random")
        {
            std::cerr << "Evaluation requires a trained model; not found: " << model << "\n";
            return 1;
        }
        std::cout << "RL agent: " << agent->DescribeParameters() << " | mode=" << cfg.mode << "\n";
    }

    std::cout << "Strategy=" << cfg.strategy << " mobility=" << cfg.mobility
              << (cfg.mobility == "sumo" ? " scenario=" + pc.scenario.name : "") << " vehicles=" << pc.numVehicles
              << " RSUs=" << cfg.numRsus << " failedRsu=" << cfg.failedRsu << " failureTime=" << cfg.failureTime
              << " episodes=" << cfg.trainingEpisodes << " states=" << encoder->GetNumStates()
              << " actions=" << cfg.numActions << "\n";

    // ---------------- TRAINING / EVALUATION LOOP ----------------
    uint64_t globalEpisode = cfg.episodeIndex;
    for (uint32_t ep = 0; ep < cfg.trainingEpisodes; ++ep)
    {
        bool last = (ep + 1 == cfg.trainingEpisodes);
        RunEpisode(pc, ep, last, agent.get(), *encoder, globalEpisode);
        if (agent && cfg.IsTraining() && (last || (cfg.saveEvery > 0 && (ep + 1) % cfg.saveEvery == 0)))
        {
            agent->SaveModel(cfg.ModelPath()); // MODEL PERSISTENCE (generic call)
        }
    }
    if (agent)
    {
        std::cout << "Final agent: " << agent->DescribeParameters() << "\n";
    }
    return 0;
}
