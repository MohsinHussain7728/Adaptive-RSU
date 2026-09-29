#ifndef UAVRL_RL_AGENT_FACTORY_H
#define UAVRL_RL_AGENT_FACTORY_H

// ============================================================
// GENERIC RL AGENT FACTORY
// Maps the --rlAlgorithm command-line value to a concrete RLAgent.
// This is the ONLY place that names concrete algorithm classes;
// adding SARSA / Double Q-learning / DQN means adding one entry here.
// ============================================================

#include "rl-agent.h"

#include <memory>
#include <string>
#include <vector>

namespace uavrl
{

/** Returns nullptr if the algorithm name is unknown. */
std::unique_ptr<RLAgent> CreateRLAgent(const std::string& algorithm, const AgentConfig& config);

std::vector<std::string> AvailableRLAlgorithms();

} // namespace uavrl

#endif
