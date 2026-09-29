#ifndef UAVRSU_RECOVERY_ENVIRONMENT_H
#define UAVRSU_RECOVERY_ENVIRONMENT_H

// ============================================================
// RL ENVIRONMENT / RECOVERY CONTROLLER (algorithm-independent)
//
// Drives the closed recovery loop on the ns-3 scheduler:
//
//   RSU failure --(detectionDelay)--> episode starts
//   repeat every stepInterval:
//     1. observe network state          (Observe)
//     2. encode state                   (StateEncoder)
//     3. strategy / RL agent -> action  (RecoveryStrategy::Decide)
//     4. UAV executes the action        (UavController::Execute)
//     5. AODV + traffic react while the UAV flies and hovers
//     6. measure performance            (NetworkMonitor window)
//     7. compute reward                 (RewardFunction)
//     8. observe next state
//     9. pass (s, a, r, s', terminal) to the strategy (RL agent learns)
//   until recovered (PDR >= pdrThreshold for recoveryHoldSteps windows)
//   or maxRecoverySteps decisions were taken.
//
// It never touches Q-values or any algorithm internals; the same loop runs
// the non-RL baselines, which makes the comparison fair.
// ============================================================

#include "env-snapshot.h"
#include "network-monitor.h"
#include "recovery-strategy.h"
#include "reward-function.h"
#include "rsu-manager.h"
#include "sim-config.h"
#include "state-encoder.h"
#include "topology-observer.h"
#include "uav-controller.h"

#include "../experiment/animation-manager.h"
#include "../experiment/results-writer.h"

#include "ns3/ipv4-address.h"
#include "ns3/node.h"

#include <map>
#include <string>

namespace uavrsu
{

struct EnvironmentComponents
{
    const SimConfig* config = nullptr;
    RsuManager* rsus = nullptr;
    UavController* uav = nullptr;
    NetworkMonitor* monitor = nullptr;
    TopologyObserver* topology = nullptr;
    StateEncoder* encoder = nullptr;
    RewardFunction* reward = nullptr;
    RecoveryStrategy* strategy = nullptr;
    AnimationManager* animation = nullptr;
    ns3::Ptr<ns3::Node> sourceNode;
    ns3::Ipv4Address destinationAddress;
    std::map<ns3::Ipv4Address, std::string> addressLabels; // for route logging
    int32_t failedRsuNodeId = -1;
    RunContext run;
    uint64_t* globalEpisodeCounter = nullptr;
};

class RecoveryEnvironment
{
  public:
    explicit RecoveryEnvironment(const EnvironmentComponents& components);

    /** Opens CSV files and schedules the failure, recovery trigger and sampling. */
    void Start();

    /** Writes recovery_results.csv and performance_results.csv after Simulator::Run. */
    void Finish(double simEnd);

    uint32_t GetEpisodesInRun() const;

  private:
    enum class Status
    {
        Normal,
        Failure,
        Recovering,
        Recovered,
        Unrecovered
    };

    struct PendingStep
    {
        EnvSnapshot before;
        uavrl::Observation state;
        uint32_t action = UavController::kStay;
        std::string decisionInfo;
        MoveResult move;
    };

    struct EpisodeStats
    {
        uint32_t indexInRun = 0;
        uint64_t globalIndex = 0;
        double startTime = 0;
        uint32_t steps = 0;
        uint32_t holdCounter = 0;
        double totalReward = 0;
        double pdrSum = 0;
        double startPdr = 0;
        double distance = 0;
        uint32_t moves = 0;
        uint32_t boundaryHits = 0;
        uint32_t explorations = 0;
    };

    // ---- environment core ----
    EnvSnapshot Observe();
    void QueryRoute(EnvSnapshot& snapshot) const;
    void OnRsuFailed(uint32_t rsuId);
    void BeginEpisode();
    void DecideAndAct(const EnvSnapshot& snapshot);
    void EvaluateStep();
    void EndEpisode(bool recovered, bool noDegradation, const EnvSnapshot& last);
    void WatchForDegradation();
    bool MeetsRecoveryCriterion(const EnvSnapshot& snapshot) const;

    // ---- logging ----
    void SampleTimeSeries();
    void SetStatus(Status status, const std::string& uavText);
    std::string StatusName() const;
    void LogTransition(const PendingStep& p,
                       const EnvSnapshot& after,
                       const RewardBreakdown& r,
                       const uavrl::Observation& next,
                       bool terminal,
                       bool truncated);
    void DumpRoutingTables(const std::string& tag);

    EnvironmentComponents m_c;
    const SimConfig& m_cfg;
    Status m_status = Status::Normal;
    PendingStep m_pending;
    EpisodeStats m_ep;
    uint32_t m_episodesInRun = 0;
    uint32_t m_degradedCounter = 0;
    bool m_stopScheduled = false;

    // Recovery bookkeeping for recovery_results.csv
    double m_failureTime = -1;
    double m_detectionTime = -1;
    bool m_groundConnectedAfterFailure = false;
    double m_firstE2eTime = -1;
    double m_firstRecoveryTime = -1;
    bool m_firstEpisodeRecovered = false;
    uint32_t m_firstEpisodeSteps = 0;
    uint32_t m_reRecoveries = 0;
    std::string m_routeAfterRecovery = "-";
    uint32_t m_hopsAfterRecovery = 0;

    CsvWriter m_actionsCsv;
    CsvWriter m_episodesCsv;
    CsvWriter m_pdrCsv;
};

} // namespace uavrsu

#endif
