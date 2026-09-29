#include "recovery-environment.h"

#include "ns3/aodv-routing-protocol.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/ipv4-routing-helper.h"
#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/simulator.h"

#include <cmath>
#include <iostream>
#include <sstream>

NS_LOG_COMPONENT_DEFINE("UavRsuRecoveryEnvironment");

namespace uavrsu
{

using namespace ns3;

RecoveryEnvironment::RecoveryEnvironment(const EnvironmentComponents& components)
    : m_c(components),
      m_cfg(*components.config)
{
}

uint32_t
RecoveryEnvironment::GetEpisodesInRun() const
{
    return m_episodesInRun;
}

void
RecoveryEnvironment::Start()
{
    const std::string dir = m_cfg.outputDir + "/";
    m_actionsCsv.Open(dir + "uav_actions.csv",
                      Concat(RunContext::Header(),
                             {"episode_in_run", "global_episode", "step", "time", "uav_x", "uav_y",
                              "state_id", "state_desc", "action", "action_name", "decision",
                              "target_x", "target_y", "move_distance", "clamped", "pdr_before",
                              "window_tx", "window_rx", "window_pdr", "delay_ms", "throughput_kbps",
                              "link_src", "link_dst", "e2e_connected", "ground_connected",
                              "dist_src_comp", "dist_dst_comp", "uav_neighbors", "route_valid",
                              "route_hops", "route_next_hop", "uav_forwarded", "r_pdr_improvement",
                              "r_pdr", "r_connectivity", "r_throughput", "r_delay", "r_loss",
                              "r_movement", "r_unnecessary", "r_boundary", "r_shaping", "r_terminal",
                              "reward", "next_state_id", "next_state_desc", "terminal", "truncated"}));
    m_episodesCsv.Open(dir + "episode_results.csv",
                       Concat(RunContext::Header(),
                              {"episode_in_run", "global_episode", "start_time", "end_time", "steps",
                               "recovered", "no_degradation", "time_from_failure_s", "episode_duration_s",
                               "total_reward", "mean_reward", "distance_m", "moves", "boundary_hits",
                               "explorations", "pdr_at_start", "mean_window_pdr", "final_window_pdr",
                               "final_uav_x", "final_uav_y", "agent"}));
    m_pdrCsv.Open(dir + "pdr_results.csv",
                  Concat(RunContext::Header(),
                         {"time", "phase", "uav_status", "window_tx", "window_rx", "window_pdr",
                          "cumulative_pdr", "delay_ms", "throughput_kbps", "echo_ratio", "uav_x",
                          "uav_y", "e2e_connected", "ground_connected", "uav_link_src",
                          "uav_link_dst", "uav_forwarded_window", "aodv_packets_window"}));

    m_c.rsus->AddFailureListener(MakeCallback(&RecoveryEnvironment::OnRsuFailed, this));
    Simulator::Schedule(Seconds(m_cfg.trafficStart), &NetworkMonitor::SetPhase, m_c.monitor, std::string("normal"));
    if (m_cfg.failedRsu >= 0)
    {
        Simulator::Schedule(Seconds(m_cfg.failureTime), &RsuManager::FailRsu, m_c.rsus,
                            static_cast<uint32_t>(m_cfg.failedRsu));
    }
    Simulator::Schedule(Seconds(m_cfg.trafficStart + 1.0), &RecoveryEnvironment::SampleTimeSeries, this);

    if (m_cfg.routingTables)
    {
        Simulator::Schedule(Seconds(std::max(0.0, m_cfg.failureTime - 0.5)),
                            &RecoveryEnvironment::DumpRoutingTables, this, std::string("before_failure"));
        Simulator::Schedule(Seconds(m_cfg.failureTime + m_cfg.detectionDelay),
                            &RecoveryEnvironment::DumpRoutingTables, this, std::string("after_failure"));
    }
    if (!m_c.strategy->DeploysUav())
    {
        Simulator::ScheduleNow(&UavController::SetRadioEnabled, m_c.uav, false);
        SetStatus(Status::Normal, "UAV: NOT DEPLOYED");
    }
}

// ============================================================
// RL ENVIRONMENT STATE GENERATION (raw observation)
// Collects measured traffic performance for the last window, the
// geometric connectivity estimate and the source's AODV route.
// ============================================================
EnvSnapshot
RecoveryEnvironment::Observe()
{
    EnvSnapshot s;
    s.time = Simulator::Now().GetSeconds();
    s.uavPos = m_c.uav->GetPosition();
    s.uavDeployed = m_c.strategy->DeploysUav();
    double end = s.time - m_cfg.deliveryGrace;
    s.window = m_c.monitor->GetWindow(end - m_cfg.measureWindow, end, s.time);
    s.conn = m_c.topology->Observe(s.uavDeployed);
    s.uavForwardedWindow = m_c.monitor->GetForwarded(m_c.uav->GetNode()->GetId(), end - m_cfg.measureWindow, s.time);
    if (m_cfg.failedRsu >= 0 && !m_c.rsus->IsActive(m_cfg.failedRsu))
    {
        s.hasFailedRsu = true;
        s.failedRsuPos = m_c.rsus->GetRsus()[m_cfg.failedRsu].position;
    }
    QueryRoute(s);
    return s;
}

void
RecoveryEnvironment::QueryRoute(EnvSnapshot& s) const
{
    Ptr<Ipv4ListRouting> list = DynamicCast<Ipv4ListRouting>(m_c.sourceNode->GetObject<Ipv4>()->GetRoutingProtocol());
    if (!list)
    {
        return;
    }
    for (uint32_t i = 0; i < list->GetNRoutingProtocols(); ++i)
    {
        int16_t priority;
        Ptr<aodv::RoutingProtocol> aodv = DynamicCast<aodv::RoutingProtocol>(list->GetRoutingProtocol(i, priority));
        if (!aodv)
        {
            continue;
        }
        std::ostringstream oss;
        aodv->PrintRoutingTable(Create<OutputStreamWrapper>(&oss), Time::S);
        std::istringstream lines(oss.str());
        std::string line;
        std::ostringstream dst;
        dst << m_c.destinationAddress;
        while (std::getline(lines, line))
        {
            std::istringstream ts(line);
            std::vector<std::string> tok;
            std::string t;
            while (ts >> t)
            {
                tok.push_back(t);
            }
            if (tok.size() >= 6 && tok[0] == dst.str() && tok[3] == "UP")
            {
                s.srcRouteValid = true;
                s.srcRouteHops = static_cast<uint32_t>(std::stoul(tok.back()));
                auto it = m_c.addressLabels.find(Ipv4Address(tok[1].c_str()));
                s.srcRouteNextHop = it != m_c.addressLabels.end() ? it->second : tok[1];
            }
        }
    }
}

bool
RecoveryEnvironment::MeetsRecoveryCriterion(const EnvSnapshot& s) const
{
    return s.window.Valid() && s.window.pdr >= m_cfg.pdrThreshold;
}

void
RecoveryEnvironment::OnRsuFailed(uint32_t rsuId)
{
    m_failureTime = Simulator::Now().GetSeconds();
    m_c.monitor->SetPhase("failure");
    SetStatus(Status::Failure, m_c.strategy->DeploysUav() ? "UAV: FAILURE DETECTED" : "UAV: NOT DEPLOYED");
    if (m_c.animation)
    {
        m_c.animation->OnRsuFailed(rsuId, m_c.rsus->GetRsus()[rsuId].node);
    }
    if (m_cfg.verbose)
    {
        std::cout << "[t=" << m_failureTime << "] RSU " << rsuId << " FAILED\n";
    }
    Simulator::Schedule(Seconds(m_cfg.detectionDelay), &RecoveryEnvironment::BeginEpisode, this);
}

void
RecoveryEnvironment::BeginEpisode()
{
    if (m_detectionTime < 0)
    {
        m_detectionTime = Simulator::Now().GetSeconds();
    }
    m_ep = EpisodeStats();
    m_ep.indexInRun = m_episodesInRun++;
    m_ep.globalIndex = m_c.globalEpisodeCounter ? (*m_c.globalEpisodeCounter)++ : m_ep.indexInRun;
    m_ep.startTime = Simulator::Now().GetSeconds();

    EnvSnapshot s = Observe();
    if (m_ep.indexInRun == 0)
    {
        m_groundConnectedAfterFailure = s.conn.groundConnected;
    }
    m_ep.startPdr = s.window.Valid() ? s.window.pdr : 0.0;
    m_c.monitor->SetPhase("recovery");
    SetStatus(Status::Recovering, "UAV: RECOVERING");
    m_c.strategy->OnEpisodeStart();

    if (m_cfg.verbose)
    {
        std::cout << "[t=" << s.time << "] EPISODE " << m_ep.globalIndex << " START | window PDR="
                  << s.window.pdr << " ground_connected=" << s.conn.groundConnected
                  << " uav=(" << s.uavPos.x << "," << s.uavPos.y << ")\n";
    }
    if (MeetsRecoveryCriterion(s))
    {
        // The failure did not degrade the monitored flow (e.g. a ground detour exists).
        EndEpisode(true, true, s);
        return;
    }
    DecideAndAct(s);
}

// ============================================================
// ACTION SELECTION + UAV ACTION EXECUTION
// ============================================================
void
RecoveryEnvironment::DecideAndAct(const EnvSnapshot& snapshot)
{
    m_pending = PendingStep();
    m_pending.before = snapshot;
    m_pending.state = m_c.encoder->Encode(snapshot);                                   // state encoding
    m_pending.action = m_c.strategy->Decide(snapshot, m_pending.state, *m_c.uav);      // RL agent / baseline selects action
    m_pending.decisionInfo = m_c.strategy->GetDecisionInfo();
    m_pending.move = m_c.uav->Execute(m_pending.action);                               // UAV executes the action

    m_ep.distance += m_pending.move.distance;
    m_ep.moves += (m_pending.move.distance > 0);
    m_ep.boundaryHits += m_pending.move.clamped;
    m_ep.explorations += (m_pending.decisionInfo == "explore");

    if (m_c.animation)
    {
        std::ostringstream os;
        os << "UAV: RECOVERING step " << (m_ep.steps + 1) << " " << UavController::ActionName(m_pending.action);
        m_c.animation->SetUavStatus(os.str(), 255, 140, 0);
    }
    // Let the UAV fly, AODV re-route and traffic flow before the next observation.
    Simulator::Schedule(Seconds(m_cfg.stepInterval), &RecoveryEnvironment::EvaluateStep, this);
}

void
RecoveryEnvironment::EvaluateStep()
{
    EnvSnapshot after = Observe(); // next state (s')
    m_ep.steps++;

    bool meets = MeetsRecoveryCriterion(after);
    m_ep.holdCounter = meets ? m_ep.holdCounter + 1 : 0;
    bool recovered = m_ep.holdCounter >= m_cfg.recoveryHoldSteps;
    bool truncated = !recovered && m_ep.steps >= m_cfg.maxRecoverySteps;

    if (after.conn.e2eConnected && !after.conn.groundConnected && m_firstE2eTime < 0)
    {
        m_firstE2eTime = after.time;
    }

    // ============================================================
    // REWARD CALCULATION (environment layer)
    // ============================================================
    RewardBreakdown r = m_c.reward->Compute(m_pending.before, after, m_pending.move, recovered, truncated);
    uavrl::Observation next = m_c.encoder->Encode(after);

    // ============================================================
    // TRANSITION -> RL AGENT (policy update happens inside the agent)
    // Terminal only for genuine recovery; truncation keeps bootstrapping.
    // ============================================================
    uavrl::Transition t;
    t.state = m_pending.state;
    t.action = m_pending.action;
    t.reward = r.total;
    t.nextState = next;
    t.terminal = recovered;
    m_c.strategy->Learn(t);

    m_ep.totalReward += r.total;
    m_ep.pdrSum += after.window.pdr;
    LogTransition(m_pending, after, r, next, recovered, truncated);

    if (recovered || truncated)
    {
        EndEpisode(recovered, false, after);
        return;
    }
    DecideAndAct(after);
}

void
RecoveryEnvironment::EndEpisode(bool recovered, bool noDegradation, const EnvSnapshot& last)
{
    m_c.strategy->OnEpisodeEnd();
    double now = Simulator::Now().GetSeconds();

    if (m_ep.indexInRun == 0)
    {
        m_firstEpisodeRecovered = recovered;
        m_firstEpisodeSteps = m_ep.steps;
        if (recovered)
        {
            m_firstRecoveryTime = now;
        }
    }
    else
    {
        m_reRecoveries++;
    }
    if (recovered)
    {
        m_routeAfterRecovery = last.srcRouteNextHop;
        m_hopsAfterRecovery = last.srcRouteHops;
    }

    CsvRow row;
    m_c.run.AppendTo(row);
    row << m_ep.indexInRun << m_ep.globalIndex << m_ep.startTime << now << m_ep.steps << recovered
        << noDegradation << (m_failureTime >= 0 ? now - m_failureTime : -1.0) << (now - m_ep.startTime)
        << m_ep.totalReward << (m_ep.steps ? m_ep.totalReward / m_ep.steps : 0.0) << m_ep.distance
        << m_ep.moves << m_ep.boundaryHits << m_ep.explorations << m_ep.startPdr
        << (m_ep.steps ? m_ep.pdrSum / m_ep.steps : last.window.pdr) << last.window.pdr << last.uavPos.x
        << last.uavPos.y << m_c.strategy->GetName();
    m_episodesCsv.Write(row);

    if (m_cfg.verbose)
    {
        std::cout << "[t=" << now << "] EPISODE " << m_ep.globalIndex << (recovered ? " RECOVERED" : " NOT RECOVERED")
                  << (noDegradation ? " (no degradation)" : "") << " | steps=" << m_ep.steps
                  << " reward=" << m_ep.totalReward << " distance=" << m_ep.distance << " m"
                  << " route_next_hop=" << last.srcRouteNextHop << "\n";
    }

    if (recovered)
    {
        m_c.monitor->SetPhase("stable");
        SetStatus(Status::Recovered, "UAV: RECOVERED");
    }
    else
    {
        m_c.monitor->SetPhase("unrecovered");
        SetStatus(Status::Unrecovered, "UAV: NOT RECOVERED (hovering)");
    }
    if (m_cfg.routingTables && m_ep.indexInRun == 0)
    {
        DumpRoutingTables(recovered ? "after_recovery" : "after_timeout");
    }

    if (m_cfg.stopAfterEpisode >= 0)
    {
        if (!m_stopScheduled)
        {
            m_stopScheduled = true;
            Simulator::Stop(Seconds(m_cfg.stopAfterEpisode));
        }
        return; // training shortcut: no re-recovery episodes
    }
    if (m_cfg.reRecovery)
    {
        m_degradedCounter = 0;
        Simulator::Schedule(Seconds(m_cfg.stepInterval), &RecoveryEnvironment::WatchForDegradation, this);
    }
}

void
RecoveryEnvironment::WatchForDegradation()
{
    // After an episode the UAV hovers; if mobility breaks connectivity again
    // (or it was never restored), a new recovery episode starts.
    EnvSnapshot s = Observe();
    bool degraded = !MeetsRecoveryCriterion(s);
    m_degradedCounter = degraded ? m_degradedCounter + 1 : 0;
    double remaining = m_cfg.simulationTime - s.time;
    if (m_degradedCounter >= m_cfg.degradationWindows && remaining > 3 * m_cfg.stepInterval)
    {
        BeginEpisode();
        return;
    }
    Simulator::Schedule(Seconds(m_cfg.stepInterval), &RecoveryEnvironment::WatchForDegradation, this);
}

void
RecoveryEnvironment::SetStatus(Status status, const std::string& uavText)
{
    m_status = status;
    if (!m_c.animation)
    {
        return;
    }
    if (!m_c.strategy->DeploysUav())
    {
        m_c.animation->SetUavStatus("UAV: NOT DEPLOYED", 150, 150, 150);
        return;
    }
    switch (status)
    {
    case Status::Recovering:
        m_c.animation->SetUavStatus(uavText, 255, 140, 0);
        break;
    case Status::Recovered:
        m_c.animation->SetUavStatus(uavText, 0, 200, 0);
        break;
    case Status::Unrecovered:
        m_c.animation->SetUavStatus(uavText, 128, 0, 128);
        break;
    default:
        m_c.animation->SetUavStatus(uavText, 0, 0, 255);
    }
}

std::string
RecoveryEnvironment::StatusName() const
{
    switch (m_status)
    {
    case Status::Normal:
        return "normal";
    case Status::Failure:
        return "failure";
    case Status::Recovering:
        return "recovering";
    case Status::Recovered:
        return "recovered";
    case Status::Unrecovered:
        return "unrecovered";
    }
    return "unknown";
}

void
RecoveryEnvironment::SampleTimeSeries()
{
    double now = Simulator::Now().GetSeconds();
    double end = now - m_cfg.deliveryGrace;
    double window = std::max(m_cfg.monitorInterval, m_cfg.measureWindow);
    WindowStats w = m_c.monitor->GetWindow(end - window, end, now);
    WindowStats cum = m_c.monitor->GetWindow(m_cfg.trafficStart, end, now);
    ConnectivityInfo c = m_c.topology->Observe(m_c.strategy->DeploysUav());
    Vector p = m_c.uav->GetPosition();

    CsvRow row;
    m_c.run.AppendTo(row);
    row << now << m_c.monitor->GetPhase() << StatusName() << w.tx << w.rx << w.pdr << cum.pdr
        << 1000.0 * w.meanDelay << w.throughputKbps << (w.tx ? static_cast<double>(w.echoRx) / w.tx : 0.0)
        << p.x << p.y << c.e2eConnected << c.groundConnected << c.uavLinkSrc << c.uavLinkDst
        << m_c.monitor->GetForwarded(m_c.uav->GetNode()->GetId(), end - window, now)
        << m_c.monitor->GetAodvPackets(now - window, now);
    m_pdrCsv.Write(row);

    Simulator::Schedule(Seconds(m_cfg.monitorInterval), &RecoveryEnvironment::SampleTimeSeries, this);
}

void
RecoveryEnvironment::LogTransition(const PendingStep& p,
                                   const EnvSnapshot& a,
                                   const RewardBreakdown& r,
                                   const uavrl::Observation& next,
                                   bool terminal,
                                   bool truncated)
{
    CsvRow row;
    m_c.run.AppendTo(row);
    row << m_ep.indexInRun << m_ep.globalIndex << m_ep.steps << p.before.time << p.before.uavPos.x
        << p.before.uavPos.y << p.state.discreteId << m_c.encoder->Describe(p.state.discreteId) << p.action
        << UavController::ActionName(p.action) << (p.decisionInfo.empty() ? "-" : p.decisionInfo)
        << p.move.target.x << p.move.target.y << p.move.distance << p.move.clamped << p.before.window.pdr
        << a.window.tx << a.window.rx << a.window.pdr << 1000.0 * a.window.meanDelay << a.window.throughputKbps
        << a.conn.uavLinkSrc << a.conn.uavLinkDst << a.conn.e2eConnected << a.conn.groundConnected
        << a.conn.distSrcComp << a.conn.distDstComp << a.conn.uavNeighbors << a.srcRouteValid << a.srcRouteHops
        << a.srcRouteNextHop << a.uavForwardedWindow << r.pdrImprovement << r.pdrLevel << r.connectivity
        << r.throughput << r.delay << r.loss << r.movement << r.unnecessaryMove << r.boundary << r.shaping
        << r.terminal << r.total << next.discreteId << m_c.encoder->Describe(next.discreteId) << terminal
        << truncated;
    m_actionsCsv.Write(row);

    if (m_cfg.verbose)
    {
        std::cout << "  step " << m_ep.steps << " t=" << a.time << " s=" << p.state.discreteId << " ["
                  << m_c.encoder->Describe(p.state.discreteId) << "] a=" << UavController::ActionName(p.action)
                  << "(" << p.decisionInfo << ") uav=(" << a.uavPos.x << "," << a.uavPos.y << ") pdr="
                  << a.window.pdr << " links=" << a.conn.uavLinkSrc << a.conn.uavLinkDst
                  << " route=" << a.srcRouteNextHop << " uavFwd=" << a.uavForwardedWindow
                  << " r=" << r.total << (terminal ? " TERMINAL" : "") << (truncated ? " TRUNCATED" : "") << "\n";
    }
}

void
RecoveryEnvironment::DumpRoutingTables(const std::string& tag)
{
    std::string path = m_cfg.outputDir + "/routing_tables/" + m_c.run.runId + "_" + tag + ".txt";
    Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>(path, std::ios::out);
    Ipv4RoutingHelper::PrintRoutingTableAllAt(Seconds(0), stream);
}

void
RecoveryEnvironment::Finish(double simEnd)
{
    // ---- performance_results.csv: phase-based metrics ----
    std::vector<std::pair<std::string, std::pair<double, double>>> extra;
    if (m_failureTime >= 0)
    {
        extra.push_back({"pre_failure", {m_cfg.trafficStart, m_failureTime}});
        extra.push_back({"post_failure", {m_failureTime, simEnd}});
    }
    extra.push_back({"overall", {m_cfg.trafficStart, simEnd}});
    auto phases = m_c.monitor->ComputePhaseStats(simEnd, m_c.uav->GetNode()->GetId(), m_c.failedRsuNodeId, extra);

    CsvWriter perf;
    perf.Open(m_cfg.outputDir + "/performance_results.csv",
              Concat(RunContext::Header(),
                     {"phase", "duration_s", "tx", "rx", "rx_timely", "pdr", "timely_pdr", "loss",
                      "throughput_kbps", "mean_delay_ms", "p95_delay_ms", "jitter_ms", "aodv_packets",
                      "aodv_bytes", "normalized_routing_load", "uav_forwarded", "failed_rsu_forwarded"}));
    auto find = [&phases](const std::string& n) -> const PhaseStats* {
        for (const auto& p : phases)
        {
            if (p.phase == n)
            {
                return &p;
            }
        }
        return nullptr;
    };
    for (const auto& p : phases)
    {
        CsvRow row;
        m_c.run.AppendTo(row);
        row << p.phase << p.duration << p.tx << p.rx << p.rxTimely << p.pdr << p.timelyPdr << p.loss
            << p.throughputKbps << p.meanDelayMs << p.p95DelayMs << p.jitterMs << p.aodvPackets << p.aodvBytes
            << p.normalizedRoutingLoad << p.uavForwarded << p.failedRsuForwarded;
        perf.Write(row);
    }

    // ---- recovery_results.csv: one row per simulation run ----
    CsvWriter rec;
    rec.Open(m_cfg.outputDir + "/recovery_results.csv",
             Concat(RunContext::Header(),
                    {"failure_time", "detection_time", "pdr_normal", "pdr_failure", "failure_impact",
                     "ground_connected_after_failure", "failed_rsu_forwarded_normal",
                     "failed_rsu_forwarded_after_failure", "first_geo_bridge_time", "first_uav_forward_time",
                     "recovered",
                     "recovery_time_s", "recovery_steps", "uav_distance_m", "uav_moves", "final_uav_x",
                     "final_uav_y", "final_dist_to_failed_rsu_m", "uav_forwarded_after_failure",
                     "route_next_hop_after_recovery", "route_hops_after_recovery", "pdr_post_failure",
                     "timely_pdr_post_failure", "pdr_stable", "episodes_in_run", "re_recoveries"}));
    const PhaseStats* normal = find("pre_failure");
    const PhaseStats* failure = find("failure");
    const PhaseStats* post = find("post_failure");
    const PhaseStats* stable = find("stable");
    Vector fp = m_c.uav->GetPosition();
    double distFailed = -1;
    if (m_cfg.failedRsu >= 0)
    {
        Vector r = m_c.rsus->GetRsus()[m_cfg.failedRsu].position;
        distFailed = std::hypot(fp.x - r.x, fp.y - r.y);
    }
    double pdrNormal = normal ? normal->timelyPdr : 0.0;
    double pdrFailure = failure ? failure->timelyPdr : 0.0;

    CsvRow row;
    m_c.run.AppendTo(row);
    row << m_failureTime << m_detectionTime << pdrNormal << pdrFailure << (pdrNormal - pdrFailure)
        << m_groundConnectedAfterFailure << (normal ? normal->failedRsuForwarded : 0)
        << (post ? post->failedRsuForwarded : 0) << m_firstE2eTime
        << m_c.monitor->GetFirstForwardTime(m_c.uav->GetNode()->GetId(), std::max(0.0, m_failureTime))
        << m_firstEpisodeRecovered
        << (m_firstEpisodeRecovered ? m_firstRecoveryTime - m_failureTime : -1.0) << m_firstEpisodeSteps
        << m_c.uav->GetTotalDistance() << m_c.uav->GetMoveCount() << fp.x << fp.y << distFailed
        << (post ? post->uavForwarded : 0) << m_routeAfterRecovery << m_hopsAfterRecovery
        << (post ? post->pdr : 0.0) << (post ? post->timelyPdr : 0.0) << (stable ? stable->timelyPdr : 0.0)
        << m_episodesInRun << m_reRecoveries;
    rec.Write(row);

    m_actionsCsv.Close();
    m_episodesCsv.Close();
    m_pdrCsv.Close();
}

} // namespace uavrsu
