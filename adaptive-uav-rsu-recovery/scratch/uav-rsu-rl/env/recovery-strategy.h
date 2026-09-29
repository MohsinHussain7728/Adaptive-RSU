#ifndef UAVRSU_RECOVERY_STRATEGY_H
#define UAVRSU_RECOVERY_STRATEGY_H

// ENVIRONMENT/EXPERIMENT LAYER - UAV recovery strategies compared in the study.
//
//   none          UAV not deployed (radio off)          -> "no UAV recovery"
//   static        UAV hovers at its initial position    -> "static UAV"
//   reactive-rsu  greedy flight toward the failed RSU's last known location
//   reactive-gap  greedy flight toward the midpoint of the narrowest gap
//                 between the source-side and destination-side components
//   rl            actions chosen by a generic uavrl::RLAgent (Q-learning by default)
//
// None of the strategies uses predefined/candidate UAV positions. All of them
// act through the same relative movement actions, so movement cost, recovery
// time and PDR are directly comparable. Only RlStrategy talks to an RL agent,
// and it does so exclusively through the generic RLAgent interface.

#include "env-snapshot.h"
#include "uav-controller.h"

#include "../rl/rl-agent.h"

#include <memory>
#include <string>

namespace uavrsu
{

class RecoveryStrategy
{
  public:
    virtual ~RecoveryStrategy() = default;
    virtual std::string GetName() const = 0;
    virtual bool DeploysUav() const
    {
        return true;
    }
    virtual void OnEpisodeStart()
    {
    }
    /** Chooses the next movement action. */
    virtual uint32_t Decide(const EnvSnapshot& snapshot,
                            const uavrl::Observation& state,
                            const UavController& uav) = 0;
    /** Receives the transition produced by the last decision (learning strategies only). */
    virtual void Learn(const uavrl::Transition& /*transition*/)
    {
    }
    virtual void OnEpisodeEnd()
    {
    }
    virtual std::string GetDecisionInfo() const
    {
        return "";
    }
};

class NoRecoveryStrategy : public RecoveryStrategy
{
  public:
    std::string GetName() const override;
    bool DeploysUav() const override;
    uint32_t Decide(const EnvSnapshot&, const uavrl::Observation&, const UavController&) override;
};

class StaticUavStrategy : public RecoveryStrategy
{
  public:
    std::string GetName() const override;
    uint32_t Decide(const EnvSnapshot&, const uavrl::Observation&, const UavController&) override;
};

/** Deterministic, non-learning reactive baseline. */
class ReactiveStrategy : public RecoveryStrategy
{
  public:
    enum class Target
    {
        FailedRsu,
        ConnectivityGap
    };

    explicit ReactiveStrategy(Target target);
    std::string GetName() const override;
    uint32_t Decide(const EnvSnapshot& snapshot, const uavrl::Observation&, const UavController& uav) override;
    std::string GetDecisionInfo() const override;

  private:
    Target m_target;
    std::string m_info;
};

/**
 * Adapter between the environment and ANY RL algorithm.
 * The strategy forwards observations/transitions to the generic agent and
 * never inspects algorithm internals (e.g. Q-values).
 */
class RlStrategy : public RecoveryStrategy
{
  public:
    explicit RlStrategy(uavrl::RLAgent* agent);
    std::string GetName() const override;
    void OnEpisodeStart() override;
    // RL AGENT SELECTS AN ACTION
    uint32_t Decide(const EnvSnapshot&, const uavrl::Observation& state, const UavController&) override;
    // RL ALGORITHM UPDATES ITS POLICY
    void Learn(const uavrl::Transition& transition) override;
    void OnEpisodeEnd() override;
    std::string GetDecisionInfo() const override;

  private:
    uavrl::RLAgent* m_agent;
};

std::unique_ptr<RecoveryStrategy> CreateRecoveryStrategy(const std::string& name, uavrl::RLAgent* agent);

} // namespace uavrsu

#endif
