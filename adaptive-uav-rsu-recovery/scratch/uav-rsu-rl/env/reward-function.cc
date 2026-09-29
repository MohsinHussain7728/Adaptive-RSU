#include "reward-function.h"

#include <algorithm>
#include <cmath>

namespace uavrsu
{

RewardFunction::RewardFunction(const RewardWeights& weights,
                               double movementStep,
                               double linkRange,
                               double nominalThroughputKbps,
                               double gamma)
    : m_w(weights),
      m_step(movementStep),
      m_range(linkRange),
      m_nominalThr(nominalThroughputKbps),
      m_gamma(gamma)
{
}

double
RewardFunction::ConnectivityScore(const ConnectivityInfo& c) const
{
    if (c.e2eConnected)
    {
        return 1.0;
    }
    return -0.4 + 0.3 * ((c.uavLinkSrc ? 1 : 0) + (c.uavLinkDst ? 1 : 0));
}

double
RewardFunction::Potential(const ConnectivityInfo& c) const
{
    auto gap = [this](double d) { return std::isfinite(d) ? std::max(0.0, d - m_range) : 10.0 * m_range; };
    return -(gap(c.distSrcComp) + gap(c.distDstComp)) / m_range;
}

RewardBreakdown
RewardFunction::Compute(const EnvSnapshot& before,
                        const EnvSnapshot& after,
                        const MoveResult& move,
                        bool recovered,
                        bool truncated) const
{
    RewardBreakdown r;
    double pdr0 = before.window.Valid() ? before.window.pdr : 0.0;
    double pdr1 = after.window.Valid() ? after.window.pdr : 0.0;

    r.pdrImprovement = m_w.pdrImprovement * (pdr1 - pdr0);
    r.pdrLevel = m_w.pdrLevel * pdr1;
    r.connectivity = m_w.connectivity * ConnectivityScore(after.conn);
    if (m_nominalThr > 0)
    {
        r.throughput = m_w.throughput * (after.window.throughputKbps - before.window.throughputKbps) / m_nominalThr;
    }
    if (after.window.rx > 0 && m_w.delayReference > 0)
    {
        r.delay = -m_w.delay * std::min(after.window.meanDelay / m_w.delayReference, 1.0);
    }
    if (after.window.Valid())
    {
        r.loss = -m_w.loss * after.window.loss;
    }
    r.movement = -m_w.movement * move.distance / m_step;
    if (before.conn.e2eConnected && move.distance > 0)
    {
        r.unnecessaryMove = -m_w.unnecessaryMove;
    }
    if (move.clamped)
    {
        r.boundary = -m_w.boundary;
    }
    // At a true terminal state the next-state potential is defined as 0.
    double phiNext = recovered ? 0.0 : Potential(after.conn);
    r.shaping = m_w.shaping * (m_gamma * phiNext - Potential(before.conn));
    if (recovered)
    {
        r.terminal = m_w.successBonus;
    }
    else if (truncated)
    {
        r.terminal = -m_w.timeoutPenalty;
    }

    r.total = r.pdrImprovement + r.pdrLevel + r.connectivity + r.throughput + r.delay + r.loss +
              r.movement + r.unnecessaryMove + r.boundary + r.shaping + r.terminal;
    return r;
}

} // namespace uavrsu
