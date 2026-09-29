#ifndef UAVRSU_STATE_ENCODER_H
#define UAVRSU_STATE_ENCODER_H

// ============================================================
// RL ENVIRONMENT - STATE GENERATION
// Converts an EnvSnapshot into the algorithm-facing uavrl::Observation
// (a discrete id for tabular agents + a feature vector for function
// approximation). Encoders are interchangeable; the state representation
// can be extended by adding a new StateEncoder without touching the agent.
// Algorithm-independent.
// ============================================================

#include "env-snapshot.h"

#include "../rl/rl-agent.h"

#include <memory>
#include <string>

namespace uavrsu
{

class StateEncoder
{
  public:
    virtual ~StateEncoder() = default;
    virtual std::string GetName() const = 0;
    virtual uint32_t GetNumStates() const = 0;
    virtual uint32_t GetNumFeatures() const = 0;
    virtual uavrl::Observation Encode(const EnvSnapshot& snapshot) const = 0;
    /** Human-readable decoding of a discrete state id (written to state_encoding.csv). */
    virtual std::string Describe(uint32_t stateId) const = 0;
};

/**
 * Default encoder: relative connectivity state.
 *
 *   srcSide : 8 = UAV is linked to the source-side ground component,
 *             otherwise the 8-sector compass direction (0=E,1=NE,2=N,...,7=SE)
 *             toward the nearest node of that component, optionally combined
 *             with a distance bin (near <= 2*range, far) when distanceBins = 2
 *   dstSide : same for the destination-side component
 *   pdrLevel: 0 = PDR < 0.2, 1 = 0.2 <= PDR < threshold, 2 = PDR >= threshold
 *
 * States = (8*bins + 1)^2 * 3   -> 243 (bins = 1) or 867 (bins = 2).
 *
 * The state is relative to the UAV (not an absolute grid cell), so a learned
 * policy transfers across failure locations, start positions and mobility traces.
 */
class ConnectivityStateEncoder : public StateEncoder
{
  public:
    ConnectivityStateEncoder(double linkRange, double pdrThreshold, uint32_t distanceBins, double areaWidth, double areaHeight);

    std::string GetName() const override;
    uint32_t GetNumStates() const override;
    uint32_t GetNumFeatures() const override;
    uavrl::Observation Encode(const EnvSnapshot& snapshot) const override;
    std::string Describe(uint32_t stateId) const override;

  private:
    uint32_t SideCode(bool linked, double bearing, double distance) const;
    std::string SideName(uint32_t code) const;

    double m_range;
    double m_pdrThreshold;
    uint32_t m_bins;
    double m_areaWidth;
    double m_areaHeight;
};

std::unique_ptr<StateEncoder> CreateStateEncoder(const std::string& name,
                                                 double linkRange,
                                                 double pdrThreshold,
                                                 uint32_t distanceBins,
                                                 double areaWidth,
                                                 double areaHeight);

} // namespace uavrsu

#endif
