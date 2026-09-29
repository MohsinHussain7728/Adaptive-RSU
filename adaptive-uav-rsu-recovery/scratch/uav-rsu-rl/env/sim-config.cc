#include "sim-config.h"

#include <sstream>

namespace uavrsu
{

void
SimConfig::Register(ns3::CommandLine& cmd)
{
    // Scenario / mobility
    cmd.AddValue("mobility", "Vehicle mobility: sumo | static", mobility);
    cmd.AddValue("sumoScenario", "SUMO scenario base directory", sumoScenario);
    cmd.AddValue("vehicleDensity", "SUMO density sub-directory: low | medium | high", vehicleDensity);
    cmd.AddValue("traceVariants", "Comma list of SUMO trace variants (episode k uses k mod n)", traceVariants);
    cmd.AddValue("traceFile", "Explicit ns-2 mobility trace (overrides scenario lookup)", traceFile);
    cmd.AddValue("numVehicles", "Number of vehicles (0 = all vehicles in scenario)", numVehicles);
    cmd.AddValue("numRsus", "Number of RSUs", numRsus);
    cmd.AddValue("sourceVehicle", "Traffic source vehicle index", sourceVehicle);
    cmd.AddValue("destinationVehicle", "Traffic destination vehicle index (-1 = last)", destinationVehicle);

    // Failure
    cmd.AddValue("failedRsu", "RSU that fails (-1 = none)", failedRsu);
    cmd.AddValue("failureTime", "RSU failure time [s]", failureTime);
    cmd.AddValue("detectionDelay", "Delay from failure to start of UAV recovery [s]", detectionDelay);

    // Time / traffic
    cmd.AddValue("simulationTime", "Simulation duration [s]", simulationTime);
    cmd.AddValue("trafficStart", "Application start time [s]", trafficStart);
    cmd.AddValue("packetInterval", "UDP echo packet interval [s]", packetInterval);
    cmd.AddValue("packetSize", "UDP echo packet size [bytes]", packetSize);
    cmd.AddValue("backgroundFlows", "Number of random background UDP flows", backgroundFlows);
    cmd.AddValue("backgroundRate", "Background flow rate [packets/s]", backgroundRate);
    cmd.AddValue("stopAfterEpisode", "If >=0, stop the run this many seconds after the episode ends", stopAfterEpisode);

    // Radio / routing
    cmd.AddValue("txPower", "Wi-Fi transmit power [dBm]", txPowerDbm);
    cmd.AddValue("commRange", "Radio range [m]", commRange);
    cmd.AddValue("pathLossExponent", "Log-distance path-loss exponent", pathLossExponent);
    cmd.AddValue("phyMode", "802.11p PHY mode", phyMode);
    cmd.AddValue("aodvHelloInterval", "AODV hello interval [s]", aodvHelloInterval);
    cmd.AddValue("aodvAllowedHelloLoss", "AODV allowed hello loss", aodvAllowedHelloLoss);
    cmd.AddValue("aodvRreqRetries", "AODV RREQ retries", aodvRreqRetries);
    cmd.AddValue("aodvNetTraversalTime", "AODV net traversal time [s]", aodvNetTraversalTime);
    cmd.AddValue("aodvActiveRouteTimeout", "AODV active route timeout [s]", aodvActiveRouteTimeout);
    cmd.AddValue("aodvMaxQueueTime", "AODV max queue time [s]", aodvMaxQueueTime);
    cmd.AddValue("staticArp", "Pre-populate ARP caches instead of dynamic ARP", staticArp);

    // UAV
    cmd.AddValue("uavX", "UAV initial x [m] (default: scenario home)", uavX);
    cmd.AddValue("uavY", "UAV initial y [m] (default: scenario home)", uavY);
    cmd.AddValue("uavZ", "UAV altitude [m]", uavZ);
    cmd.AddValue("randomUavStart", "Draw the UAV start position uniformly in the operational area", randomUavStart);
    cmd.AddValue("uavSpeed", "UAV flight speed [m/s]", uavSpeed);
    cmd.AddValue("movementStep", "UAV displacement per movement action [m]", movementStep);
    cmd.AddValue("numActions", "Action set size: 5 (N,S,E,W,Stay) or 9 (+diagonals)", numActions);
    cmd.AddValue("areaMinX", "Operational area min x [m]", areaMinX);
    cmd.AddValue("areaMaxX", "Operational area max x [m]", areaMaxX);
    cmd.AddValue("areaMinY", "Operational area min y [m]", areaMinY);
    cmd.AddValue("areaMaxY", "Operational area max y [m]", areaMaxY);

    // Recovery
    cmd.AddValue("strategy", "Recovery strategy: none | static | reactive-rsu | reactive-gap | rl", strategy);
    cmd.AddValue("mode", "train | eval", mode);
    cmd.AddValue("maxRecoverySteps", "Maximum UAV decisions per recovery episode", maxRecoverySteps);
    cmd.AddValue("stepInterval", "Time between UAV decisions [s]", stepInterval);
    cmd.AddValue("measureWindow", "Traffic window evaluated per observation [s]", measureWindow);
    cmd.AddValue("deliveryGrace", "Max delay for a packet to count as delivered in a window [s]", deliveryGrace);
    cmd.AddValue("pdrThreshold", "Windowed PDR that counts as recovered", pdrThreshold);
    cmd.AddValue("recoveryHoldSteps", "Consecutive windows above threshold required", recoveryHoldSteps);
    cmd.AddValue("reRecovery", "Start a new episode if PDR degrades again after recovery", reRecovery);
    cmd.AddValue("degradationWindows", "Consecutive degraded windows that trigger re-recovery", degradationWindows);
    cmd.AddValue("linkMargin", "Fraction of commRange the observer treats as a reliable link", linkMargin);
    cmd.AddValue("stateEncoder", "State encoder: connectivity", stateEncoder);
    cmd.AddValue("stateDistanceBins", "Distance bins per direction in the state (1 or 2)", stateDistanceBins);

    // RL
    cmd.AddValue("rlAlgorithm", "RL algorithm (qlearning | random | ...)", rlAlgorithm);
    cmd.AddValue("alpha", "Learning rate", alpha);
    cmd.AddValue("gamma", "Discount factor", gamma);
    cmd.AddValue("epsilon", "Initial exploration rate", epsilon);
    cmd.AddValue("epsilonMin", "Minimum exploration rate", epsilonMin);
    cmd.AddValue("epsilonDecay", "Per-episode multiplicative epsilon decay", epsilonDecay);
    cmd.AddValue("evalEpsilon", "Exploration rate in evaluation mode", evalEpsilon);
    cmd.AddValue("initQ", "Initial Q-value", initQ);
    cmd.AddValue("episodeIndex", "Global index of the first episode run by this process", episodeIndex);
    cmd.AddValue("trainingEpisodes", "Episodes (independent simulation runs) executed by this process", trainingEpisodes);
    cmd.AddValue("modelFile", "Model/Q-table file (default <outputDir>/uav_qtable.csv)", modelFile);
    cmd.AddValue("saveEvery", "Checkpoint model every N episodes", saveEvery);

    // Reward weights
    cmd.AddValue("wPdrImprovement", "Reward weight: PDR improvement", reward.pdrImprovement);
    cmd.AddValue("wPdr", "Reward weight: PDR level", reward.pdrLevel);
    cmd.AddValue("wConnectivity", "Reward weight: connectivity", reward.connectivity);
    cmd.AddValue("wThroughput", "Reward weight: throughput improvement", reward.throughput);
    cmd.AddValue("wDelay", "Reward weight: delay penalty", reward.delay);
    cmd.AddValue("wLoss", "Reward weight: packet-loss penalty", reward.loss);
    cmd.AddValue("wMove", "Reward weight: movement cost", reward.movement);
    cmd.AddValue("wUnnecessaryMove", "Reward weight: unnecessary movement", reward.unnecessaryMove);
    cmd.AddValue("wBoundary", "Reward weight: leaving operational area", reward.boundary);
    cmd.AddValue("wShaping", "Reward weight: potential-based distance shaping", reward.shaping);
    cmd.AddValue("successBonus", "Terminal reward for recovery", reward.successBonus);
    cmd.AddValue("timeoutPenalty", "Penalty when maxRecoverySteps is reached", reward.timeoutPenalty);

    // Output
    cmd.AddValue("outputDir", "Directory for CSV/XML outputs", outputDir);
    cmd.AddValue("runLabel", "Free-text label stored in every CSV row", runLabel);
    cmd.AddValue("netanim", "Write NetAnim XML (last episode only)", netanim);
    cmd.AddValue("animFile", "NetAnim file name", animFile);
    cmd.AddValue("flowmonXml", "Write FlowMonitor XML", flowmonXml);
    cmd.AddValue("routingTables", "Dump AODV routing tables at key instants", routingTables);
    cmd.AddValue("monitorInterval", "Time-series sampling interval [s]", monitorInterval);
    cmd.AddValue("seed", "ns-3 RNG seed", seed);
    cmd.AddValue("run", "ns-3 RNG run number (episode k uses run + k)", run);
    cmd.AddValue("verbose", "Print per-step log to stdout", verbose);
    cmd.AddValue("diagnostics", "Print per-node forwarding/drop counters after each run", diagnostics);
}

std::string
SimConfig::Validate() const
{
    std::ostringstream err;
    if (mobility != "sumo" && mobility != "static")
    {
        err << "--mobility must be sumo or static. ";
    }
    if (strategy != "none" && strategy != "static" && strategy != "reactive-rsu" &&
        strategy != "reactive-gap" && strategy != "rl")
    {
        err << "--strategy must be none|static|reactive-rsu|reactive-gap|rl. ";
    }
    if (mode != "train" && mode != "eval")
    {
        err << "--mode must be train or eval. ";
    }
    if (numActions != 5 && numActions != 9)
    {
        err << "--numActions must be 5 or 9. ";
    }
    if (numRsus < 1)
    {
        err << "--numRsus must be >= 1. ";
    }
    if (failedRsu >= static_cast<int32_t>(numRsus))
    {
        err << "--failedRsu must be < numRsus. ";
    }
    if (movementStep <= 0 || uavSpeed <= 0)
    {
        err << "--movementStep and --uavSpeed must be positive. ";
    }
    if (movementStep / uavSpeed + measureWindow + deliveryGrace > stepInterval + 1e-9)
    {
        err << "--stepInterval must cover flight time + measureWindow + deliveryGrace. ";
    }
    if (stateDistanceBins < 1 || stateDistanceBins > 2)
    {
        err << "--stateDistanceBins must be 1 or 2. ";
    }
    if (pdrThreshold <= 0 || pdrThreshold > 1)
    {
        err << "--pdrThreshold must be in (0,1]. ";
    }
    return err.str();
}

std::string
SimConfig::ModelPath() const
{
    return modelFile.empty() ? outputDir + "/uav_qtable.csv" : modelFile;
}

} // namespace uavrsu
