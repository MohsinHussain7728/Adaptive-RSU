#ifndef UAVRL_QLEARNING_AGENT_H
#define UAVRL_QLEARNING_AGENT_H

// ============================================================
// Q-LEARNING IMPLEMENTATION
// This component implements the current RL algorithm.
// It should be replaceable without modifying the simulation.
//
// Only this class owns and manipulates:
//   - the Q-table and visit counts
//   - epsilon-greedy exploration
//   - action selection
//   - the Q-value update rule
//   - model persistence (uav_qtable.csv)
// The simulation only holds an RLAgent pointer and never sees Q-values.
// ============================================================

#include "rl-agent.h"

#include <random>
#include <string>
#include <vector>

namespace uavrl
{

/**
 * Tabular Q-learning with epsilon-greedy exploration.
 *
 *   Q(s,a) <- Q(s,a) + alpha * [ r + gamma * max_a' Q(s',a') - Q(s,a) ]   (non-terminal)
 *   Q(s,a) <- Q(s,a) + alpha * [ r - Q(s,a) ]                            (terminal)
 *
 * Recognised AgentConfig::params (all optional):
 *   alpha         learning rate                          (default 0.1)
 *   gamma         discount factor                        (default 0.9)
 *   epsilon       initial exploration rate               (default 0.2)
 *   epsilonMin    lower bound for decayed epsilon        (default 0.0)
 *   epsilonDecay  multiplicative decay per episode       (default 1.0 = none)
 *   episodeIndex  global index of the first episode run by this process; lets an
 *                 external training loop continue the decay schedule across processes
 *   evalEpsilon   exploration used in evaluation mode    (default 0.0 = greedy)
 *   initQ         initial Q-value for unseen pairs       (default 0.0)
 */
class QLearningAgent : public RLAgent
{
  public:
    explicit QLearningAgent(const AgentConfig& config);

    std::string GetName() const override;
    uint32_t SelectAction(const Observation& state) override;
    void Update(const Transition& transition) override;
    void SetTrainingMode(bool training) override;
    bool IsTraining() const override;
    void OnEpisodeStart() override;
    void OnEpisodeEnd() override;
    bool SaveModel(const std::string& path) const override;
    bool LoadModel(const std::string& path) override;
    void Reset() override;
    std::string DescribeParameters() const override;
    std::string GetLastDecisionInfo() const override;

    // Algorithm-specific inspection used by unit tests only; the simulation
    // interacts exclusively through the RLAgent interface above.
    double GetQValue(uint32_t state, uint32_t action) const;
    double GetCurrentEpsilon() const;
    uint64_t GetEpisodesTrained() const;

  private:
    uint32_t GreedyAction(uint32_t state);
    double MaxQ(uint32_t state) const;
    std::size_t Index(uint32_t state, uint32_t action) const;
    void RefreshEpsilon();

    uint32_t m_numStates;
    uint32_t m_numActions;
    double m_alpha;
    double m_gamma;
    double m_epsilon0;
    double m_epsilonMin;
    double m_epsilonDecay;
    double m_evalEpsilon;
    double m_initQ;
    uint64_t m_episodeOffset;

    double m_epsilon;
    bool m_training;
    uint64_t m_episodesTrained;
    uint64_t m_episodesThisProcess;
    std::string m_lastDecision;

    std::vector<double> m_q;
    std::vector<uint64_t> m_visits;
    std::mt19937_64 m_rng;
};

} // namespace uavrl

#endif
