#ifndef UAVRSU_SCENARIO_H
#define UAVRSU_SCENARIO_H

// Scenario geometry shared by the SUMO generator and the ns-3 program.
//
// Urban layout (default values, all configurable in scenario_info.cfg):
//
//      y=400 +-----+-----+ . . . . . . . . . +-----+-----+
//            |     |     |                   |     |     |
//            +-----R0----+       R2 (bridge  R3----+-----+
//            |     |     R1      RSU in the  |     |     R4
//      y=0   +-----+-----+ . . . river/park) +-----+-----+
//            x=0      x=400               x=800      x=1200
//            West district   400 m gap    East district
//
// Vehicles circulate inside their district (source vehicle in the West,
// destination vehicle in the East). The only infrastructure link across the
// gap is the bridge RSU, so its failure partitions the network. This is a
// deliberate design choice: the failure must have a real, verifiable impact
// instead of AODV silently re-routing over an unaffected ground path.

#include "ns3/vector.h"

#include <cstdint>
#include <string>
#include <vector>

namespace uavrsu
{

struct ScenarioGeometry
{
    double blockLength = 200.0;
    uint32_t districtBlocks = 2;
    double gapWidth = 400.0;

    double DistrictWidth() const
    {
        return blockLength * districtBlocks;
    }

    double Height() const
    {
        return blockLength * districtBlocks;
    }

    double WestMinX() const
    {
        return 0.0;
    }

    double WestMaxX() const
    {
        return DistrictWidth();
    }

    double GapCenterX() const
    {
        return DistrictWidth() + gapWidth / 2.0;
    }

    double EastMinX() const
    {
        return DistrictWidth() + gapWidth;
    }

    double EastMaxX() const
    {
        return 2.0 * DistrictWidth() + gapWidth;
    }
};

struct ScenarioInfo
{
    std::string name = "builtin";
    std::string directory;
    ScenarioGeometry geometry;
    uint32_t numVehicles = 20;
    bool loadedFromFile = false;
};

/** Reads <dir>/scenario_info.cfg (key=value). Missing file -> built-in defaults. */
ScenarioInfo LoadScenarioInfo(const std::string& dir);

/** RSU positions: West RSUs, one bridge RSU (index numRsus/2) in the gap, East RSUs. */
std::vector<ns3::Vector> ComputeRsuLayout(const ScenarioGeometry& geo, uint32_t numRsus);

/** Reads id,x,y,z rows; returns an empty vector if the file is absent. */
std::vector<ns3::Vector> LoadRsuFile(const std::string& path);

/** Default UAV home: above the West district (not in the gap). */
ns3::Vector DefaultUavHome(const ScenarioGeometry& geo, double z);

/**
 * Deterministic, stationary vehicle layout on the district roads used by
 * --mobility=static for debugging the recovery mechanism without SUMO.
 * The first half of the vehicles is placed in the West district, the rest in
 * the East district; vehicle 0 and vehicle n-1 sit at the far corners.
 */
std::vector<ns3::Vector> StaticVehicleLayout(const ScenarioGeometry& geo, uint32_t numVehicles);

/** Index of the bridge RSU for a given RSU count. */
uint32_t BridgeRsuIndex(uint32_t numRsus);

} // namespace uavrsu

#endif
