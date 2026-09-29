#ifndef UAVRSU_UAV_CONTROLLER_H
#define UAVRSU_UAV_CONTROLLER_H

// ENVIRONMENT LAYER - UAV ACTION EXECUTION.
// Translates an abstract action index (chosen by any strategy or RL agent)
// into a physical flight of the UAV node. There are NO predefined or candidate
// recovery positions: every action is a relative displacement of
// movementStep metres from wherever the UAV currently is, and the flight is
// performed at a finite speed (ConstantVelocityMobilityModel) so that the
// network keeps evolving during the move. Algorithm-independent.
//
// Action space (numActions = 9; with numActions = 5 only A0..A4 exist):
//   A0 North  A1 South  A2 East  A3 West  A4 Stay
//   A5 North-East  A6 North-West  A7 South-East  A8 South-West
// Diagonal moves have the same length as axial moves.

#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/event-id.h"
#include "ns3/node.h"
#include "ns3/vector.h"

#include <string>

namespace uavrsu
{

struct OperationalArea
{
    double minX = 0;
    double maxX = 0;
    double minY = 0;
    double maxY = 0;
};

struct MoveResult
{
    uint32_t action = 4;
    ns3::Vector from;
    ns3::Vector target;
    double distance = 0.0;   // metres actually flown
    double flightTime = 0.0; // seconds
    bool clamped = false;    // requested move would have left the operational area
};

class UavController
{
  public:
    static constexpr uint32_t kStay = 4;

    void Install(ns3::Ptr<ns3::Node> uav,
                 const ns3::Vector& start,
                 const OperationalArea& area,
                 double movementStep,
                 double speed,
                 uint32_t numActions);

    uint32_t GetNumActions() const;
    static std::string ActionName(uint32_t action);

    /** Unit direction (x,y) of an action; (0,0) for Stay. */
    static ns3::Vector ActionDirection(uint32_t action);

    /** Where the action would take the UAV (clamped to the area). */
    ns3::Vector PredictTarget(uint32_t action, bool* clamped) const;

    /** Executes the action: starts a constant-speed flight toward the target. */
    MoveResult Execute(uint32_t action);

    ns3::Vector GetPosition() const;
    ns3::Ptr<ns3::Node> GetNode() const;
    const OperationalArea& GetArea() const;
    double GetMovementStep() const;
    double GetTotalDistance() const;
    uint32_t GetMoveCount() const;

    /** Switch the UAV radio off ("no UAV recovery" baseline) or on. */
    void SetRadioEnabled(bool enabled);
    bool IsRadioEnabled() const;

  private:
    void Arrive(ns3::Vector target);

    ns3::Ptr<ns3::Node> m_node;
    ns3::Ptr<ns3::ConstantVelocityMobilityModel> m_mobility;
    OperationalArea m_area;
    double m_step = 40.0;
    double m_speed = 20.0;
    uint32_t m_numActions = 9;
    double m_totalDistance = 0.0;
    uint32_t m_moveCount = 0;
    bool m_radioEnabled = true;
    ns3::EventId m_arrival;
};

} // namespace uavrsu

#endif
