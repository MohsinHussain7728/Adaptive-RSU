#ifndef UAVRSU_SIM_CONFIG_H
#define UAVRSU_SIM_CONFIG_H

// Every experiment parameter lives here and is exposed on the command line
// (see SimConfig::Register). Nothing else in the code base hard-codes
// scenario, radio, recovery, RL or reward parameters.

#include "ns3/command-line.h"

#include <cmath>
#include <cstdint>
#include <string>

namespace uavrsu
{

struct RewardWeights
{
    double pdrImprovement = 2.0; // * (PDR_t+1 - PDR_t)
    double pdrLevel = 0.5;       // * PDR_t+1
    double connectivity = 1.0;   // * connectivity score (end-to-end link through UAV)
    double throughput = 0.5;     // * normalised throughput improvement
    double delay = 0.3;          // * normalised end-to-end delay
    double loss = 0.5;           // * packet-loss ratio
    double movement = 0.1;       // * distance moved / movementStep
    double unnecessaryMove = 0.3; // moving although end-to-end connectivity already existed
    double boundary = 0.5;       // action tried to leave the operational area
    double shaping = 1.0;        // potential-based distance shaping (0 disables)
    double successBonus = 5.0;   // terminal: communication recovered
    double timeoutPenalty = 2.0; // episode truncated at maxRecoverySteps
    double delayReference = 0.1; // s, delay normalisation
};

struct SimConfig
{
    // ---------------- scenario / mobility ----------------
    std::string mobility = "sumo";                     // "sumo" | "static" (debug layout)
    std::string sumoScenario = "scratch/sumo-scenario"; // scenario base directory
    std::string vehicleDensity = "low";                // low | medium | high (sub-directory)
    std::string traceVariants = "1";                   // comma list; episode k uses variant k % n
    std::string traceFile;                             // explicit ns-2 trace (overrides above)
    uint32_t numVehicles = 0;                          // 0 = all vehicles of the scenario
    uint32_t numRsus = 5;
    uint32_t sourceVehicle = 0;
    int32_t destinationVehicle = -1; // -1 = last vehicle

    // ---------------- RSU failure ----------------
    int32_t failedRsu = 2;       // -1 = no failure
    double failureTime = 20.0;   // s
    double detectionDelay = 1.0; // s between failure and start of recovery (heartbeat loss)

    // ---------------- time / traffic ----------------
    double simulationTime = 120.0;
    double trafficStart = 2.0;
    double packetInterval = 0.05; // s (UDP echo, Vehicle src -> Vehicle dst)
    uint32_t packetSize = 1024;   // bytes
    uint32_t backgroundFlows = 0; // extra random vehicle-to-vehicle UDP flows
    double backgroundRate = 2.0;  // packets/s per background flow
    double stopAfterEpisode = -1; // >=0: end the simulation this long after the (last) episode

    // ---------------- radio / routing ----------------
    double txPowerDbm = 23.0;
    double commRange = 250.0;      // m, hard radio range (RangePropagationLossModel)
    double pathLossExponent = 2.0; // log-distance exponent inside the range
    std::string phyMode = "OfdmRate6MbpsBW10MHz";
    double aodvHelloInterval = 1.0;
    uint32_t aodvAllowedHelloLoss = 2;
    uint32_t aodvRreqRetries = 2;
    double aodvNetTraversalTime = 2.8; // s (ns-3 default)
    double aodvActiveRouteTimeout = 3.0;
    double aodvMaxQueueTime = 30.0;
    bool staticArp = true; // pre-populated ARP caches (see uav-rsu-rl.cc)

    // ---------------- UAV ----------------
    double uavX = NAN; // NAN = scenario default home position
    double uavY = NAN;
    double uavZ = 50.0;
    bool randomUavStart = false;
    double uavSpeed = 20.0;     // m/s
    double movementStep = 40.0; // m per action
    uint32_t numActions = 9;    // 5 (N,S,E,W,Stay) or 9 (+ diagonals)
    double areaMinX = NAN;
    double areaMaxX = NAN;
    double areaMinY = NAN;
    double areaMaxY = NAN;

    // ---------------- recovery control ----------------
    std::string strategy = "rl"; // none | static | reactive-rsu | reactive-gap | rl
    std::string mode = "train";  // train | eval
    uint32_t maxRecoverySteps = 25;
    double stepInterval = 3.0;  // s between consecutive decisions (flight + settle + measure)
    double measureWindow = 0.8; // s of traffic evaluated per observation
    double deliveryGrace = 0.2; // s a packet may take to count as delivered in time
    double pdrThreshold = 0.8;
    uint32_t recoveryHoldSteps = 1; // consecutive windows above threshold to declare recovery
    bool reRecovery = true;         // start a new episode if the network degrades again
    uint32_t degradationWindows = 2;
    double linkMargin = 0.95; // fraction of commRange assumed reliable by the observer
    std::string stateEncoder = "connectivity";
    uint32_t stateDistanceBins = 1;

    // ---------------- RL (generic; forwarded to the agent as a parameter map) ----------------
    std::string rlAlgorithm = "qlearning";
    double alpha = 0.1;
    double gamma = 0.9;
    double epsilon = 0.2;
    double epsilonMin = 0.02;
    double epsilonDecay = 1.0;
    double evalEpsilon = 0.0;
    double initQ = 0.0;
    uint32_t episodeIndex = 0;     // global index of first episode (decay schedule)
    uint32_t trainingEpisodes = 1; // simulation runs (episodes) executed by this process
    std::string modelFile;         // default: <outputDir>/uav_qtable.csv
    uint32_t saveEvery = 10;       // checkpoint the model every N episodes

    RewardWeights reward;

    // ---------------- output ----------------
    std::string outputDir = "results";
    std::string runLabel;
    bool netanim = true;
    std::string animFile = "uav-rsu-rl-animation.xml";
    bool flowmonXml = true;
    bool routingTables = true;
    double monitorInterval = 0.5;
    uint32_t seed = 1;
    uint32_t run = 1;
    bool verbose = true;
    bool diagnostics = false;

    void Register(ns3::CommandLine& cmd);
    /** Returns an empty string if valid, otherwise an error message. */
    std::string Validate() const;
    std::string ModelPath() const;
    bool IsTraining() const
    {
        return mode == "train";
    }
    bool UsesRl() const
    {
        return strategy == "rl";
    }
};

} // namespace uavrsu

#endif
