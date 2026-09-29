#include "rl-agent-factory.h"

#include "qlearning-agent.h"
#include "random-agent.h"

namespace uavrl
{

std::unique_ptr<RLAgent>
CreateRLAgent(const std::string& algorithm, const AgentConfig& config)
{
    if (algorithm == "qlearning")
    {
        return std::make_unique<QLearningAgent>(config);
    }
    if (algorithm == "random")
    {
        return std::make_unique<RandomAgent>(config);
    }
    // Future algorithms are registered here, e.g.:
    //   if (algorithm == "sarsa")        return std::make_unique<SarsaAgent>(config);
    //   if (algorithm == "doubleq")      return std::make_unique<DoubleQLearningAgent>(config);
    //   if (algorithm == "dqn")          return std::make_unique<DqnAgent>(config);
    return nullptr;
}

std::vector<std::string>
AvailableRLAlgorithms()
{
    return {"qlearning", "random"};
}

} // namespace uavrl
