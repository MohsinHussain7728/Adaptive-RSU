#ifndef UAVRSU_REWARD_FUNCTION_H
#define UAVRSU_REWARD_FUNCTION_H

// ============================================================
// RL ENVIRONMENT - REWARD CALCULATION
// The reward belongs to the environment, NOT to the RL algorithm.
// Any agent (Q-learning, SARSA, DQN, ...) receives the same scalar
// reward through uavrl::Transition. Algorithm-independent.
//
//   r =  wPdrImprovement * (PDR' - PDR)
//      + wPdr            * PDR'
//      + wConnectivity   * connectivityScore'
//      + wThroughput     * (Thr' - Thr) / nominalThroughput
//      - wDelay          * min(delay' / delayReference, 1)
//      - wLoss           * loss'
//      - wMove           * distanceMoved / movementStep
//      - wUnnecessary    * [moved although end-to-end connectivity existed]
//      - wBoundary       * [action tried to leave the operational area]
//      + wShaping        * (gamma * Phi(s') - Phi(s))     potential-based shaping
//      + successBonus    * [recovered]      (terminal)
//      - timeoutPenalty  * [step limit hit] (truncation)
//
// connectivityScore = 1 if src-dst are connected (via the UAV), otherwise
// -0.4 + 0.3 * (#components the UAV is linked to).
// Phi(s) = -(gapSrc + gapDst) / range, gapX = max(0, distance to component X - range).
// Potential-based shaping preserves the optimal policy (Ng et al., 1999)
// and can be disabled with --wShaping=0 for ablation.
// ============================================================

#include "env-snapshot.h"
#include "sim-config.h"
#include "uav-controller.h"

namespace uavrsu
{

struct RewardBreakdown
{
    double pdrImprovement = 0;
    double pdrLevel = 0;
    double connectivity = 0;
    double throughput = 0;
    double delay = 0;
    double loss = 0;
    double movement = 0;
    double unnecessaryMove = 0;
    double boundary = 0;
    double shaping = 0;
    double terminal = 0;
    double total = 0;
};

class RewardFunction
{
  public:
    RewardFunction(const RewardWeights& weights,
                   double movementStep,
                   double linkRange,
                   double nominalThroughputKbps,
                   double gamma);

    RewardBreakdown Compute(const EnvSnapshot& before,
                            const EnvSnapshot& after,
                            const MoveResult& move,
                            bool recovered,
                            bool truncated) const;

    double ConnectivityScore(const ConnectivityInfo& c) const;
    double Potential(const ConnectivityInfo& c) const;

  private:
    RewardWeights m_w;
    double m_step;
    double m_range;
    double m_nominalThr;
    double m_gamma;
};

} // namespace uavrsu

#endif
