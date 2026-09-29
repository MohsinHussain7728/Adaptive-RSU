#include "recovery-strategy.h"

#include <cmath>
#include <limits>
#include <sstream>

namespace uavrsu
{

using ns3::Vector;

std::string
NoRecoveryStrategy::GetName() const
{
    return "none";
}

bool
NoRecoveryStrategy::DeploysUav() const
{
    return false;
}

uint32_t
NoRecoveryStrategy::Decide(const EnvSnapshot&, const uavrl::Observation&, const UavController&)
{
    return UavController::kStay;
}

std::string
StaticUavStrategy::GetName() const
{
    return "static";
}

uint32_t
StaticUavStrategy::Decide(const EnvSnapshot&, const uavrl::Observation&, const UavController&)
{
    return UavController::kStay;
}

ReactiveStrategy::ReactiveStrategy(Target target)
    : m_target(target)
{
}

std::string
ReactiveStrategy::GetName() const
{
    return m_target == Target::FailedRsu ? "reactive-rsu" : "reactive-gap";
}

uint32_t
ReactiveStrategy::Decide(const EnvSnapshot& snap, const uavrl::Observation&, const UavController& uav)
{
    Vector target;
    if (m_target == Target::ConnectivityGap)
    {
        if (snap.conn.e2eConnected || !snap.conn.gapValid)
        {
            m_info = "hold-connected";
            return UavController::kStay;
        }
        target = snap.conn.gapMidpoint;
        m_info = "gap";
    }
    else
    {
        if (!snap.hasFailedRsu)
        {
            m_info = "no-failure";
            return UavController::kStay;
        }
        target = snap.failedRsuPos;
        m_info = "failed-rsu";
    }

    auto horizontal = [&target](const Vector& p) { return std::hypot(p.x - target.x, p.y - target.y); };
    double current = horizontal(uav.GetPosition());
    if (current <= uav.GetMovementStep() / 2.0)
    {
        m_info += "-reached";
        return UavController::kStay;
    }
    // Greedy one-step look-ahead over the same relative action set the RL agent uses.
    uint32_t best = UavController::kStay;
    double bestDist = current;
    for (uint32_t a = 0; a < uav.GetNumActions(); ++a)
    {
        double d = horizontal(uav.PredictTarget(a, nullptr));
        if (d < bestDist - 1e-9)
        {
            bestDist = d;
            best = a;
        }
    }
    return best;
}

std::string
ReactiveStrategy::GetDecisionInfo() const
{
    return m_info;
}

RlStrategy::RlStrategy(uavrl::RLAgent* agent)
    : m_agent(agent)
{
}

std::string
RlStrategy::GetName() const
{
    return "rl";
}

void
RlStrategy::OnEpisodeStart()
{
    m_agent->OnEpisodeStart();
}

// ============================================================
// RL AGENT SELECTS AN ACTION (generic interface call)
// ============================================================
uint32_t
RlStrategy::Decide(const EnvSnapshot&, const uavrl::Observation& state, const UavController&)
{
    return m_agent->SelectAction(state);
}

// ============================================================
// RL ALGORITHM UPDATES ITS POLICY (generic interface call)
// The agent itself decides whether to learn (training) or not (evaluation).
// ============================================================
void
RlStrategy::Learn(const uavrl::Transition& transition)
{
    m_agent->Update(transition);
}

void
RlStrategy::OnEpisodeEnd()
{
    m_agent->OnEpisodeEnd();
}

std::string
RlStrategy::GetDecisionInfo() const
{
    return m_agent->GetLastDecisionInfo();
}

std::unique_ptr<RecoveryStrategy>
CreateRecoveryStrategy(const std::string& name, uavrl::RLAgent* agent)
{
    if (name == "none")
    {
        return std::make_unique<NoRecoveryStrategy>();
    }
    if (name == "static")
    {
        return std::make_unique<StaticUavStrategy>();
    }
    if (name == "reactive-rsu")
    {
        return std::make_unique<ReactiveStrategy>(ReactiveStrategy::Target::FailedRsu);
    }
    if (name == "reactive-gap")
    {
        return std::make_unique<ReactiveStrategy>(ReactiveStrategy::Target::ConnectivityGap);
    }
    if (name == "rl" && agent)
    {
        return std::make_unique<RlStrategy>(agent);
    }
    return nullptr;
}

} // namespace uavrsu
