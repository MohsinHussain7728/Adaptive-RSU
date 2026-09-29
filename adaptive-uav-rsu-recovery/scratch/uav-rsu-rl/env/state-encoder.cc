#include "state-encoder.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace uavrsu
{

namespace
{
const char* kSectors[] = {"E", "NE", "N", "NW", "W", "SW", "S", "SE"};

uint32_t
Sector(double bearing)
{
    const double pi = 3.14159265358979323846;
    double a = std::fmod(bearing + 2 * pi + pi / 8, 2 * pi);
    return static_cast<uint32_t>(a / (pi / 4)) % 8;
}
} // namespace

ConnectivityStateEncoder::ConnectivityStateEncoder(double linkRange,
                                                   double pdrThreshold,
                                                   uint32_t distanceBins,
                                                   double areaWidth,
                                                   double areaHeight)
    : m_range(linkRange),
      m_pdrThreshold(pdrThreshold),
      m_bins(distanceBins),
      m_areaWidth(areaWidth),
      m_areaHeight(areaHeight)
{
}

std::string
ConnectivityStateEncoder::GetName() const
{
    return "connectivity";
}

uint32_t
ConnectivityStateEncoder::GetNumStates() const
{
    uint32_t side = 8 * m_bins + 1;
    return side * side * 3;
}

uint32_t
ConnectivityStateEncoder::GetNumFeatures() const
{
    return 11;
}

uint32_t
ConnectivityStateEncoder::SideCode(bool linked, double bearing, double distance) const
{
    uint32_t linkedCode = 8 * m_bins;
    if (linked)
    {
        return linkedCode;
    }
    uint32_t bin = (m_bins == 2 && distance > 2.0 * m_range) ? 1 : 0;
    return bin * 8 + Sector(bearing);
}

std::string
ConnectivityStateEncoder::SideName(uint32_t code) const
{
    if (code == 8 * m_bins)
    {
        return "LINKED";
    }
    std::string s = kSectors[code % 8];
    if (m_bins == 2)
    {
        s += (code >= 8) ? "-far" : "-near";
    }
    return s;
}

uavrl::Observation
ConnectivityStateEncoder::Encode(const EnvSnapshot& snap) const
{
    const ConnectivityInfo& c = snap.conn;
    uint32_t side = 8 * m_bins + 1;
    uint32_t src = SideCode(c.uavLinkSrc, c.bearingSrc, c.distSrcComp);
    uint32_t dst = SideCode(c.uavLinkDst, c.bearingDst, c.distDstComp);
    double pdr = snap.window.Valid() ? snap.window.pdr : 0.0;
    uint32_t level = pdr < 0.2 ? 0 : (pdr < m_pdrThreshold ? 1 : 2);

    uavrl::Observation obs;
    obs.discreteId = (src * side + dst) * 3 + level;

    // Continuous features for function-approximation agents (e.g. DQN).
    auto clip = [](double v) { return std::isfinite(v) ? std::min(v, 10.0) : 10.0; };
    obs.features = {std::cos(c.bearingSrc),
                    std::sin(c.bearingSrc),
                    clip(c.distSrcComp / m_range),
                    std::cos(c.bearingDst),
                    std::sin(c.bearingDst),
                    clip(c.distDstComp / m_range),
                    c.uavLinkSrc ? 1.0 : 0.0,
                    c.uavLinkDst ? 1.0 : 0.0,
                    pdr,
                    m_areaWidth > 0 ? snap.uavPos.x / m_areaWidth : 0.0,
                    m_areaHeight > 0 ? snap.uavPos.y / m_areaHeight : 0.0};
    return obs;
}

std::string
ConnectivityStateEncoder::Describe(uint32_t id) const
{
    uint32_t side = 8 * m_bins + 1;
    uint32_t level = id % 3;
    uint32_t pair = id / 3;
    uint32_t dst = pair % side;
    uint32_t src = pair / side;
    static const char* levels[] = {"PDR_LOW", "PDR_MID", "PDR_OK"};
    std::ostringstream os;
    os << "src=" << SideName(src) << ";dst=" << SideName(dst) << ";" << levels[level];
    return os.str();
}

std::unique_ptr<StateEncoder>
CreateStateEncoder(const std::string& name,
                   double linkRange,
                   double pdrThreshold,
                   uint32_t distanceBins,
                   double areaWidth,
                   double areaHeight)
{
    if (name == "connectivity")
    {
        return std::make_unique<ConnectivityStateEncoder>(linkRange, pdrThreshold, distanceBins, areaWidth, areaHeight);
    }
    return nullptr;
}

} // namespace uavrsu
