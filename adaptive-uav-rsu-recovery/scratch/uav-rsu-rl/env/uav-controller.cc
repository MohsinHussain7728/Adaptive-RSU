#include "uav-controller.h"

#include "wifi-radio.h"

#include "ns3/simulator.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy.h"

#include <algorithm>
#include <cmath>

namespace uavrsu
{

using namespace ns3;

void
UavController::Install(Ptr<Node> uav,
                       const Vector& start,
                       const OperationalArea& area,
                       double movementStep,
                       double speed,
                       uint32_t numActions)
{
    m_node = uav;
    m_area = area;
    m_step = movementStep;
    m_speed = speed;
    m_numActions = numActions;
    m_mobility = CreateObject<ConstantVelocityMobilityModel>();
    m_mobility->SetPosition(start);
    m_mobility->SetVelocity(Vector(0, 0, 0));
    uav->AggregateObject(m_mobility);
}

uint32_t
UavController::GetNumActions() const
{
    return m_numActions;
}

std::string
UavController::ActionName(uint32_t action)
{
    static const char* names[] = {"North", "South", "East", "West", "Stay",
                                  "NorthEast", "NorthWest", "SouthEast", "SouthWest"};
    return action < 9 ? names[action] : "Unknown";
}

Vector
UavController::ActionDirection(uint32_t action)
{
    const double d = 1.0 / std::sqrt(2.0);
    switch (action)
    {
    case 0:
        return Vector(0, 1, 0);
    case 1:
        return Vector(0, -1, 0);
    case 2:
        return Vector(1, 0, 0);
    case 3:
        return Vector(-1, 0, 0);
    case 5:
        return Vector(d, d, 0);
    case 6:
        return Vector(-d, d, 0);
    case 7:
        return Vector(d, -d, 0);
    case 8:
        return Vector(-d, -d, 0);
    default:
        return Vector(0, 0, 0);
    }
}

Vector
UavController::PredictTarget(uint32_t action, bool* clamped) const
{
    Vector p = GetPosition();
    Vector dir = ActionDirection(action);
    Vector t(p.x + dir.x * m_step, p.y + dir.y * m_step, p.z);
    Vector c(std::clamp(t.x, m_area.minX, m_area.maxX), std::clamp(t.y, m_area.minY, m_area.maxY), p.z);
    if (clamped)
    {
        *clamped = std::fabs(c.x - t.x) > 1e-9 || std::fabs(c.y - t.y) > 1e-9;
    }
    return c;
}

// ============================================================
// UAV EXECUTES THE ACTION
// Starts a constant-speed flight toward the relative target. The
// arrival event stops the UAV exactly at the target so that the
// next observation is taken from a hovering position.
// ============================================================
MoveResult
UavController::Execute(uint32_t action)
{
    if (m_arrival.IsPending())
    {
        // Previous flight still running (stepInterval too short): stop where we are.
        m_arrival.Cancel();
        m_mobility->SetVelocity(Vector(0, 0, 0));
    }

    MoveResult r;
    r.action = action;
    r.from = GetPosition();
    r.target = PredictTarget(action, &r.clamped);
    r.distance = CalculateDistance(r.from, r.target);

    if (r.distance < 1e-6)
    {
        return r; // Stay, or a move fully blocked by the area boundary
    }
    r.flightTime = r.distance / m_speed;
    Vector v((r.target.x - r.from.x) / r.flightTime, (r.target.y - r.from.y) / r.flightTime, 0.0);
    m_mobility->SetVelocity(v);
    m_arrival = Simulator::Schedule(Seconds(r.flightTime), &UavController::Arrive, this, r.target);
    m_totalDistance += r.distance;
    m_moveCount++;
    return r;
}

void
UavController::Arrive(Vector target)
{
    m_mobility->SetVelocity(Vector(0, 0, 0));
    m_mobility->SetPosition(target);
}

Vector
UavController::GetPosition() const
{
    return m_mobility->GetPosition();
}

Ptr<Node>
UavController::GetNode() const
{
    return m_node;
}

const OperationalArea&
UavController::GetArea() const
{
    return m_area;
}

double
UavController::GetMovementStep() const
{
    return m_step;
}

double
UavController::GetTotalDistance() const
{
    return m_totalDistance;
}

uint32_t
UavController::GetMoveCount() const
{
    return m_moveCount;
}

void
UavController::SetRadioEnabled(bool enabled)
{
    m_radioEnabled = enabled;
    for (uint32_t i = 0; i < m_node->GetNDevices(); ++i)
    {
        Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(m_node->GetDevice(i));
        if (!dev)
        {
            continue;
        }
        if (enabled)
        {
            dev->GetPhy()->ResumeFromOff();
        }
        else
        {
            SwitchOffPhyWhenIdle(dev->GetPhy());
        }
    }
}

bool
UavController::IsRadioEnabled() const
{
    return m_radioEnabled;
}

} // namespace uavrsu
