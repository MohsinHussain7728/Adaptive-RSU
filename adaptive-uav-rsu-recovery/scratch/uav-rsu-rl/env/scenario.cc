#include "scenario.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace uavrsu
{

using ns3::Vector;

ScenarioInfo
LoadScenarioInfo(const std::string& dir)
{
    ScenarioInfo info;
    info.directory = dir;
    std::ifstream in(dir + "/scenario_info.cfg");
    if (!in)
    {
        return info;
    }
    info.loadedFromFile = true;
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        auto eq = line.find('=');
        if (eq == std::string::npos)
        {
            continue;
        }
        std::string key = line.substr(0, eq);
        std::string value = line.substr(eq + 1);
        if (key == "name")
        {
            info.name = value;
        }
        else if (key == "num_vehicles")
        {
            info.numVehicles = static_cast<uint32_t>(std::stoul(value));
        }
        else if (key == "block_length")
        {
            info.geometry.blockLength = std::stod(value);
        }
        else if (key == "district_blocks")
        {
            info.geometry.districtBlocks = static_cast<uint32_t>(std::stoul(value));
        }
        else if (key == "gap_width")
        {
            info.geometry.gapWidth = std::stod(value);
        }
    }
    return info;
}

uint32_t
BridgeRsuIndex(uint32_t numRsus)
{
    return numRsus / 2;
}

std::vector<Vector>
ComputeRsuLayout(const ScenarioGeometry& geo, uint32_t numRsus)
{
    std::vector<Vector> pos;
    const double h = geo.Height();
    const uint32_t bridge = BridgeRsuIndex(numRsus);
    const uint32_t west = bridge;
    const uint32_t east = numRsus - bridge - 1;

    auto lerp = [](Vector a, Vector b, double t) {
        return Vector(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, 0.0);
    };

    // West RSUs: from the district centre towards the south part of the gap edge.
    Vector wA(geo.WestMaxX() / 2.0, h / 2.0, 0.0);
    Vector wB(geo.WestMaxX(), h / 4.0, 0.0);
    for (uint32_t i = 0; i < west; ++i)
    {
        double t = (west == 1) ? 1.0 : static_cast<double>(i) / (west - 1);
        pos.push_back(lerp(wA, wB, t));
    }

    pos.emplace_back(geo.GapCenterX(), h / 2.0, 0.0);

    // East RSUs: from the north part of the gap edge towards the district centre.
    Vector eA(geo.EastMinX(), 3.0 * h / 4.0, 0.0);
    Vector eB(geo.EastMinX() + geo.DistrictWidth() / 2.0, h / 2.0, 0.0);
    for (uint32_t j = 0; j < east; ++j)
    {
        double t = (east == 1) ? 0.0 : static_cast<double>(j) / (east - 1);
        pos.push_back(lerp(eA, eB, t));
    }
    return pos;
}

std::vector<Vector>
LoadRsuFile(const std::string& path)
{
    std::vector<Vector> pos;
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#' || line.rfind("id", 0) == 0)
        {
            continue;
        }
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> v;
        while (std::getline(ss, cell, ','))
        {
            v.push_back(std::stod(cell));
        }
        if (v.size() >= 4)
        {
            pos.emplace_back(v[1], v[2], v[3]);
        }
    }
    return pos;
}

Vector
DefaultUavHome(const ScenarioGeometry& geo, double z)
{
    return Vector(geo.WestMaxX() / 2.0, 0.75 * geo.Height(), z);
}

namespace
{

// Points on the district road grid every blockLength/4 metres.
std::vector<Vector>
RoadPoints(const ScenarioGeometry& geo, double xOffset)
{
    std::vector<Vector> pts;
    const double w = geo.DistrictWidth();
    const double h = geo.Height();
    const double d = geo.blockLength / 4.0;
    for (uint32_t r = 0; r <= geo.districtBlocks; ++r)
    {
        double y = r * geo.blockLength;
        for (double x = 0.0; x <= w + 1e-6; x += d)
        {
            pts.emplace_back(xOffset + x, y, 0.0);
        }
    }
    for (uint32_t c = 0; c <= geo.districtBlocks; ++c)
    {
        double x = c * geo.blockLength;
        for (double y = d; y < h - 1e-6; y += d)
        {
            if (std::fmod(y, geo.blockLength) > 1e-6)
            {
                pts.emplace_back(xOffset + x, y, 0.0);
            }
        }
    }
    return pts;
}

std::vector<Vector>
NearestPoints(std::vector<Vector> pts, const Vector& anchor, uint32_t count)
{
    std::stable_sort(pts.begin(), pts.end(), [&anchor](const Vector& a, const Vector& b) {
        return CalculateDistance(a, anchor) < CalculateDistance(b, anchor);
    });
    std::vector<Vector> out;
    for (uint32_t i = 0; i < count; ++i)
    {
        // More vehicles than road points: stack additional vehicles 5 m apart.
        Vector p = pts[i % pts.size()];
        p.x += 5.0 * (i / pts.size());
        out.push_back(p);
    }
    return out;
}

} // namespace

std::vector<Vector>
StaticVehicleLayout(const ScenarioGeometry& geo, uint32_t numVehicles)
{
    const uint32_t west = (numVehicles + 1) / 2;
    const uint32_t east = numVehicles - west;
    // West cluster grows from the far north-west corner (source side),
    // East cluster from the far south-east corner (destination side).
    auto w = NearestPoints(RoadPoints(geo, geo.WestMinX()), Vector(geo.WestMinX(), geo.Height(), 0), west);
    auto e = NearestPoints(RoadPoints(geo, geo.EastMinX()), Vector(geo.EastMaxX(), 0.0, 0.0), east);
    std::vector<Vector> all = w;
    // Reverse so that the last vehicle is the far south-east corner.
    all.insert(all.end(), e.rbegin(), e.rend());
    return all;
}

} // namespace uavrsu
