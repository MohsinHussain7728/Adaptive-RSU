// Standalone unit test for the RL layer. It links ONLY rl/*.cc (no ns-3),
// which demonstrates that the RL algorithms are decoupled from the simulation.
//
// Build/run (inside the toolchain distro):
//   g++ -std=c++17 -O2 -I.. test_rl_agents.cpp ../rl/*.cc -o test_rl_agents && ./test_rl_agents
// (The file uses the .cpp extension on purpose so the ns-3 scratch scanner ignores it.)

#include "../rl/qlearning-agent.h"
#include "../rl/rl-agent-factory.h"

#include <cmath>
#include <cstdio>
#include <memory>

using namespace uavrl;

static int g_failures = 0;

#define CHECK(cond, msg)                                                                           \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            std::printf("FAIL: %s\n", msg);                                                        \
            g_failures++;                                                                          \
        }                                                                                          \
        else                                                                                       \
        {                                                                                          \
            std::printf("ok  : %s\n", msg);                                                        \
        }                                                                                          \
    } while (0)

// 1-D corridor: states 0..N-1, action 0 = left, 1 = right, 2 = stay.
// Reaching state N-1 is terminal with +10, every step costs -1.
static double
RunCorridorEpisode(RLAgent& agent, uint32_t n, uint32_t maxSteps)
{
    uint32_t s = 0;
    double total = 0.0;
    agent.OnEpisodeStart();
    for (uint32_t k = 0; k < maxSteps; ++k)
    {
        Observation obs;
        obs.discreteId = s;
        uint32_t a = agent.SelectAction(obs);
        uint32_t s2 = s;
        if (a == 0 && s > 0)
        {
            s2 = s - 1;
        }
        else if (a == 1)
        {
            s2 = s + 1;
        }
        bool terminal = (s2 == n - 1);
        double r = terminal ? 10.0 : -1.0;
        Observation next;
        next.discreteId = s2;
        agent.Update({obs, a, r, next, terminal});
        total += r;
        s = s2;
        if (terminal)
        {
            break;
        }
    }
    agent.OnEpisodeEnd();
    return total;
}

int
main()
{
    const uint32_t n = 8;
    AgentConfig cfg;
    cfg.numStates = n;
    cfg.numActions = 3;
    cfg.seed = 42;
    cfg.params = {{"alpha", 0.5}, {"gamma", 0.9}, {"epsilon", 0.2}};

    // Exact update rule on a fresh table.
    {
        QLearningAgent q(cfg);
        Observation s0, s1;
        s0.discreteId = 0;
        s1.discreteId = 1;
        q.Update({s0, 1, 1.0, s1, true});
        CHECK(std::fabs(q.GetQValue(0, 1) - 0.5) < 1e-12, "terminal update Q += alpha*(r - Q)");
        q.Update({s1, 1, 0.0, s0, false});
        // target = 0 + 0.9 * max Q(s0,.) = 0.45 -> Q = 0.5*0.45
        CHECK(std::fabs(q.GetQValue(1, 1) - 0.225) < 1e-12,
              "non-terminal update Q += alpha*(r + gamma*maxQ' - Q)");
    }

    // Learning through the generic factory/interface only.
    auto agent = CreateRLAgent("qlearning", cfg);
    CHECK(agent != nullptr, "factory creates qlearning");
    CHECK(CreateRLAgent("does-not-exist", cfg) == nullptr, "factory rejects unknown algorithm");
    for (int ep = 0; ep < 300; ++ep)
    {
        RunCorridorEpisode(*agent, n, 50);
    }
    agent->SetTrainingMode(false);
    double greedyReturn = RunCorridorEpisode(*agent, n, 50);
    CHECK(std::fabs(greedyReturn - (10.0 - (n - 2))) < 1e-9, "greedy policy is optimal on corridor");

    // Evaluation mode must not change the model.
    CHECK(agent->SaveModel("test_qtable_a.csv"), "save model");
    RunCorridorEpisode(*agent, n, 50);
    CHECK(agent->SaveModel("test_qtable_b.csv"), "save model after eval episode");
    {
        std::FILE* fa = std::fopen("test_qtable_a.csv", "rb");
        std::FILE* fb = std::fopen("test_qtable_b.csv", "rb");
        bool same = fa && fb;
        int ca = 0, cb = 0;
        while (same && (ca = std::fgetc(fa)) != EOF)
        {
            cb = std::fgetc(fb);
            same = (ca == cb);
        }
        same = same && std::fgetc(fb) == EOF;
        if (fa)
        {
            std::fclose(fa);
        }
        if (fb)
        {
            std::fclose(fb);
        }
        CHECK(same, "evaluation mode leaves Q-table unchanged");
    }

    // Persistence round-trip.
    auto restored = CreateRLAgent("qlearning", cfg);
    CHECK(restored->LoadModel("test_qtable_a.csv"), "load model");
    restored->SetTrainingMode(false);
    CHECK(std::fabs(RunCorridorEpisode(*restored, n, 50) - greedyReturn) < 1e-9,
          "restored model reproduces greedy return");

    AgentConfig other = cfg;
    other.numStates = n + 1;
    auto mismatched = CreateRLAgent("qlearning", other);
    CHECK(!mismatched->LoadModel("test_qtable_a.csv"), "load rejects mismatched state space");

    std::remove("test_qtable_a.csv");
    std::remove("test_qtable_b.csv");

    std::printf(g_failures == 0 ? "ALL RL TESTS PASSED\n" : "RL TESTS FAILED\n");
    return g_failures == 0 ? 0 : 1;
}
