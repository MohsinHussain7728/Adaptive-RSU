#ifndef UAVRL_RL_AGENT_H
#define UAVRL_RL_AGENT_H

// ============================================================
// GENERIC RL AGENT INTERFACE
// The simulation communicates with RL algorithms through this
// abstraction so Q-learning can later be replaced by another
// reinforcement-learning algorithm.
//
// This header (and everything in rl/) is intentionally free of
// ns-3, SUMO, FlowMonitor, UAV and RSU concepts. An agent only
// sees integer/feature observations, integer actions and scalar
// rewards. State generation, action execution and reward
// calculation live in the environment layer (env/).
//
// How to add a new algorithm (e.g. SARSA, Double Q-learning, DQN):
//   1. Implement a class deriving from RLAgent in rl/.
//   2. Register it in rl/rl-agent-factory.cc under a name.
//   3. Run the simulation with --rlAlgorithm=<name>.
// Nothing in the environment, mobility, traffic, SUMO or metrics
// code has to change.
// ============================================================

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace uavrl
{

/**
 * Observation handed to an agent. It carries two views of the same state so
 * that both tabular and function-approximation algorithms can be plugged in:
 *  - discreteId : index in [0, numStates) for tabular methods (Q-learning, SARSA)
 *  - features   : normalised real-valued vector for DQN-style methods
 */
struct Observation
{
    uint32_t discreteId = 0;
    std::vector<double> features;
};

/**
 * One environment transition (s, a, r, s').
 *
 * terminal is true only for a genuine terminal state (communication recovered).
 * Reaching the step limit is a time-limit truncation, not a terminal state, so
 * the environment reports terminal = false and bootstrapping remains valid.
 */
struct Transition
{
    Observation state;
    uint32_t action = 0;
    double reward = 0.0;
    Observation nextState;
    bool terminal = false;
};

/**
 * Algorithm-independent construction parameters. Algorithm-specific
 * hyper-parameters are passed through the generic "params" map so the
 * simulation never needs to know which hyper-parameters an algorithm has.
 */
struct AgentConfig
{
    uint32_t numStates = 0;
    uint32_t numFeatures = 0;
    uint32_t numActions = 0;
    uint64_t seed = 1;
    std::map<std::string, double> params;

    double Get(const std::string& key, double defaultValue) const;
};

class RLAgent
{
  public:
    virtual ~RLAgent() = default;

    /** Short algorithm identifier, e.g. "qlearning". */
    virtual std::string GetName() const = 0;

    /** Choose an action for the given state (may explore when training). */
    virtual uint32_t SelectAction(const Observation& state) = 0;

    /** Learn from one transition. Must be a no-op in evaluation mode. */
    virtual void Update(const Transition& transition) = 0;

    /** Training: exploration + learning. Evaluation: frozen, (near-)greedy policy. */
    virtual void SetTrainingMode(bool training) = 0;
    virtual bool IsTraining() const = 0;

    /** Episode hooks (e.g. eligibility traces, target-network sync, counters). */
    virtual void OnEpisodeStart()
    {
    }

    virtual void OnEpisodeEnd()
    {
    }

    /** Persist / restore the learned policy. */
    virtual bool SaveModel(const std::string& path) const = 0;
    virtual bool LoadModel(const std::string& path) = 0;

    /** Discard everything learned so far. */
    virtual void Reset() = 0;

    /** Human-readable hyper-parameter summary for logs. */
    virtual std::string DescribeParameters() const
    {
        return "";
    }

    /** Optional tag describing the last decision (e.g. "explore"/"greedy"), logged per step. */
    virtual std::string GetLastDecisionInfo() const
    {
        return "";
    }
};

} // namespace uavrl

#endif
