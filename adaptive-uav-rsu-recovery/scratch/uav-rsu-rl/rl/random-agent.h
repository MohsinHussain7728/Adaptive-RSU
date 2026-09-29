#ifndef UAVRL_RANDOM_AGENT_H
#define UAVRL_RANDOM_AGENT_H

#include "rl-agent.h"

#include <random>

namespace uavrl
{

/**
 * Uniform random policy. It never learns and serves two purposes:
 *  - a "random-walk UAV" sanity baseline (--rlAlgorithm=random), and
 *  - a minimal example of plugging a second algorithm into the RLAgent
 *    interface without touching the simulation.
 */
class RandomAgent : public RLAgent
{
  public:
    explicit RandomAgent(const AgentConfig& config);

    std::string GetName() const override;
    uint32_t SelectAction(const Observation& state) override;
    void Update(const Transition& transition) override;
    void SetTrainingMode(bool training) override;
    bool IsTraining() const override;
    bool SaveModel(const std::string& path) const override;
    bool LoadModel(const std::string& path) override;
    void Reset() override;
    std::string DescribeParameters() const override;
    std::string GetLastDecisionInfo() const override;

  private:
    uint32_t m_numActions;
    bool m_training;
    std::mt19937_64 m_rng;
};

} // namespace uavrl

#endif
