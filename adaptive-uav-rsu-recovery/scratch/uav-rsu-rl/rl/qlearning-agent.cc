// ============================================================
// Q-LEARNING IMPLEMENTATION
// This component implements the current RL algorithm.
// It should be replaceable without modifying the simulation.
// ============================================================

#include "qlearning-agent.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace uavrl
{

QLearningAgent::QLearningAgent(const AgentConfig& config)
    : m_numStates(config.numStates),
      m_numActions(config.numActions),
      m_alpha(config.Get("alpha", 0.1)),
      m_gamma(config.Get("gamma", 0.9)),
      m_epsilon0(config.Get("epsilon", 0.2)),
      m_epsilonMin(config.Get("epsilonMin", 0.0)),
      m_epsilonDecay(config.Get("epsilonDecay", 1.0)),
      m_evalEpsilon(config.Get("evalEpsilon", 0.0)),
      m_initQ(config.Get("initQ", 0.0)),
      m_episodeOffset(static_cast<uint64_t>(config.Get("episodeIndex", 0.0))),
      m_epsilon(m_epsilon0),
      m_training(true),
      m_episodesTrained(0),
      m_episodesThisProcess(0),
      m_rng(config.seed)
{
    Reset();
}

std::string
QLearningAgent::GetName() const
{
    return "qlearning";
}

std::size_t
QLearningAgent::Index(uint32_t state, uint32_t action) const
{
    return static_cast<std::size_t>(state) * m_numActions + action;
}

void
QLearningAgent::Reset()
{
    m_q.assign(static_cast<std::size_t>(m_numStates) * m_numActions, m_initQ);
    m_visits.assign(m_q.size(), 0);
    m_episodesTrained = 0;
    RefreshEpsilon();
}

void
QLearningAgent::RefreshEpsilon()
{
    // Exponential decay indexed by the global episode number so a schedule can be
    // continued across separate simulation processes that share one Q-table file.
    double k = static_cast<double>(m_episodeOffset + m_episodesThisProcess);
    m_epsilon = std::max(m_epsilonMin, m_epsilon0 * std::pow(m_epsilonDecay, k));
}

double
QLearningAgent::MaxQ(uint32_t state) const
{
    double best = -std::numeric_limits<double>::infinity();
    for (uint32_t a = 0; a < m_numActions; ++a)
    {
        best = std::max(best, m_q[Index(state, a)]);
    }
    return best;
}

uint32_t
QLearningAgent::GreedyAction(uint32_t state)
{
    // Ties are broken uniformly at random; otherwise an untrained table would
    // always pick action 0 and bias both learning and evaluation.
    double best = MaxQ(state);
    std::vector<uint32_t> ties;
    for (uint32_t a = 0; a < m_numActions; ++a)
    {
        if (std::fabs(m_q[Index(state, a)] - best) < 1e-12)
        {
            ties.push_back(a);
        }
    }
    std::uniform_int_distribution<std::size_t> pick(0, ties.size() - 1);
    return ties[pick(m_rng)];
}

uint32_t
QLearningAgent::SelectAction(const Observation& state)
{
    uint32_t s = state.discreteId % m_numStates;
    double eps = m_training ? m_epsilon : m_evalEpsilon;

    std::uniform_real_distribution<double> coin(0.0, 1.0);
    if (coin(m_rng) < eps)
    {
        std::uniform_int_distribution<uint32_t> any(0, m_numActions - 1);
        m_lastDecision = "explore";
        return any(m_rng);
    }

    bool unseen = true;
    for (uint32_t a = 0; a < m_numActions; ++a)
    {
        unseen = unseen && m_visits[Index(s, a)] == 0;
    }
    m_lastDecision = unseen ? "greedy-unvisited" : "greedy";
    return GreedyAction(s);
}

void
QLearningAgent::Update(const Transition& t)
{
    if (!m_training)
    {
        return; // evaluation mode: the policy is frozen
    }
    uint32_t s = t.state.discreteId % m_numStates;
    uint32_t s2 = t.nextState.discreteId % m_numStates;
    std::size_t idx = Index(s, t.action);

    double target = t.terminal ? t.reward : t.reward + m_gamma * MaxQ(s2);
    m_q[idx] += m_alpha * (target - m_q[idx]);
    m_visits[idx]++;
}

void
QLearningAgent::SetTrainingMode(bool training)
{
    m_training = training;
}

bool
QLearningAgent::IsTraining() const
{
    return m_training;
}

void
QLearningAgent::OnEpisodeStart()
{
    RefreshEpsilon();
}

void
QLearningAgent::OnEpisodeEnd()
{
    if (m_training)
    {
        m_episodesTrained++;
        m_episodesThisProcess++;
        RefreshEpsilon();
    }
}

bool
QLearningAgent::SaveModel(const std::string& path) const
{
    std::ofstream out(path);
    if (!out)
    {
        return false;
    }
    out << "# algorithm=qlearning"
        << ",num_states=" << m_numStates << ",num_actions=" << m_numActions
        << ",alpha=" << m_alpha << ",gamma=" << m_gamma << ",epsilon0=" << m_epsilon0
        << ",epsilon_current=" << m_epsilon << ",episodes_trained=" << m_episodesTrained << "\n";
    out << "state";
    for (uint32_t a = 0; a < m_numActions; ++a)
    {
        out << ",q_a" << a;
    }
    for (uint32_t a = 0; a < m_numActions; ++a)
    {
        out << ",visits_a" << a;
    }
    out << ",best_action,total_visits\n";
    out << std::setprecision(10);
    for (uint32_t s = 0; s < m_numStates; ++s)
    {
        out << s;
        double best = MaxQ(s);
        int bestAction = -1;
        uint64_t total = 0;
        for (uint32_t a = 0; a < m_numActions; ++a)
        {
            out << "," << m_q[Index(s, a)];
            if (bestAction < 0 && m_q[Index(s, a)] == best)
            {
                bestAction = static_cast<int>(a);
            }
        }
        for (uint32_t a = 0; a < m_numActions; ++a)
        {
            out << "," << m_visits[Index(s, a)];
            total += m_visits[Index(s, a)];
        }
        out << "," << (total == 0 ? -1 : bestAction) << "," << total << "\n";
    }
    return static_cast<bool>(out);
}

bool
QLearningAgent::LoadModel(const std::string& path)
{
    std::ifstream in(path);
    if (!in)
    {
        return false;
    }
    std::string line;
    std::vector<double> q(m_q.size(), m_initQ);
    std::vector<uint64_t> visits(m_q.size(), 0);
    uint64_t episodes = 0;
    bool headerSeen = false;

    while (std::getline(in, line))
    {
        if (line.empty())
        {
            continue;
        }
        if (line[0] == '#')
        {
            auto check = [&line](const std::string& key, uint64_t expected) {
                auto pos = line.find(key + "=");
                if (pos == std::string::npos)
                {
                    return true;
                }
                return std::stoull(line.substr(pos + key.size() + 1)) == expected;
            };
            if (!check("num_states", m_numStates) || !check("num_actions", m_numActions))
            {
                return false; // model was trained with a different state/action space
            }
            auto pos = line.find("episodes_trained=");
            if (pos != std::string::npos)
            {
                episodes = std::stoull(line.substr(pos + 17));
            }
            continue;
        }
        if (!headerSeen)
        {
            headerSeen = true; // column header row
            continue;
        }
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> cells;
        while (std::getline(ss, cell, ','))
        {
            cells.push_back(cell);
        }
        if (cells.size() < 1 + 2 * static_cast<std::size_t>(m_numActions))
        {
            return false;
        }
        uint32_t s = static_cast<uint32_t>(std::stoul(cells[0]));
        if (s >= m_numStates)
        {
            return false;
        }
        for (uint32_t a = 0; a < m_numActions; ++a)
        {
            q[Index(s, a)] = std::stod(cells[1 + a]);
            visits[Index(s, a)] = std::stoull(cells[1 + m_numActions + a]);
        }
    }
    m_q = q;
    m_visits = visits;
    m_episodesTrained = episodes;
    return true;
}

std::string
QLearningAgent::DescribeParameters() const
{
    std::ostringstream os;
    os << "Q-learning(alpha=" << m_alpha << ", gamma=" << m_gamma << ", epsilon0=" << m_epsilon0
       << ", epsilonMin=" << m_epsilonMin << ", epsilonDecay=" << m_epsilonDecay
       << ", currentEpsilon=" << m_epsilon << ", evalEpsilon=" << m_evalEpsilon
       << ", states=" << m_numStates << ", actions=" << m_numActions
       << ", episodesTrained=" << m_episodesTrained << ")";
    return os.str();
}

std::string
QLearningAgent::GetLastDecisionInfo() const
{
    return m_lastDecision;
}

double
QLearningAgent::GetQValue(uint32_t state, uint32_t action) const
{
    return m_q[Index(state, action)];
}

double
QLearningAgent::GetCurrentEpsilon() const
{
    return m_training ? m_epsilon : m_evalEpsilon;
}

uint64_t
QLearningAgent::GetEpisodesTrained() const
{
    return m_episodesTrained;
}

} // namespace uavrl
