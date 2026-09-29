#include "rl-agent.h"

namespace uavrl
{

double
AgentConfig::Get(const std::string& key, double defaultValue) const
{
    auto it = params.find(key);
    return it == params.end() ? defaultValue : it->second;
}

} // namespace uavrl
