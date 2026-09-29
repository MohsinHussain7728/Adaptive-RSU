#include "random-agent.h"

#include <fstream>
#include <sstream>

namespace uavrl
{

RandomAgent::RandomAgent(const AgentConfig& config)
    : m_numActions(config.numActions),
      m_training(true),
      m_rng(config.seed)
{
}

std::string
RandomAgent::GetName() const
{
    return "random";
}

uint32_t
RandomAgent::SelectAction(const Observation& /*state*/)
{
    std::uniform_int_distribution<uint32_t> any(0, m_numActions - 1);
    return any(m_rng);
}

void
RandomAgent::Update(const Transition& /*transition*/)
{
}

void
RandomAgent::SetTrainingMode(bool training)
{
    m_training = training;
}

bool
RandomAgent::IsTraining() const
{
    return m_training;
}

bool
RandomAgent::SaveModel(const std::string& path) const
{
    std::ofstream out(path);
    out << "# algorithm=random,num_actions=" << m_numActions << "\n";
    return static_cast<bool>(out);
}

bool
RandomAgent::LoadModel(const std::string& /*path*/)
{
    return true; // stateless policy
}

void
RandomAgent::Reset()
{
}

std::string
RandomAgent::DescribeParameters() const
{
    std::ostringstream os;
    os << "Random(actions=" << m_numActions << ")";
    return os.str();
}

std::string
RandomAgent::GetLastDecisionInfo() const
{
    return "random";
}

} // namespace uavrl
