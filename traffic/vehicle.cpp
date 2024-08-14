#include "vehicle.h"
#include "OpenDriveMap.h"
#include <qgraphicsscene.h>
#include <math.h>

#include "spdlog/spdlog.h"

extern QGraphicsScene* g_scene;

Vehicle::Vehicle(odr::LaneKey initialLane, double initialLocalS) :
    currKey(initialLane), s(initialLocalS), odoMeter(0),
    currLaneLength(0), tOffset(0)
{
    graphics = new QGraphicsRectItem(QRectF(-2.3, -0.9, 4.6, 1.8));
    graphics->setPen(Qt::NoPen);
    auto randColor = static_cast<Qt::GlobalColor>(rand() % static_cast<int>(Qt::GlobalColor::transparent));
    graphics->setBrush(QBrush(randColor, Qt::SolidPattern));
    g_scene->addItem(graphics);
    graphics->hide();
}

void Vehicle::Clear()
{
    g_scene->removeItem(graphics);
}

bool Vehicle::Step(double dt, const odr::OpenDriveMap& odrMap, const odr::RoutingGraph& routingGraph)
{
    if (odoMeter == 0)
    {
        // First step
        updateCurrKeyLength(odrMap);
    }
    double remainingS = currLaneLength - s;
    double laneChangeRate = tOffset / remainingS * 1.5;
    tOffset = tOffset - laneChangeRate * dt * DefaultVelocity;
    odoMeter += dt * DefaultVelocity;
    s += dt * DefaultVelocity;
    if (s > currLaneLength)
    {
        auto choices = routingGraph.get_lane_successors(currKey);
        if (choices.size() == 0)
        {
            return false;
        }
        s -= currLaneLength;
        currKey = choices.at(rand() % choices.size());
        updateCurrKeyLength(odrMap);
        if (currLaneLength == 0)
        {
            return false;
        }
        checkedForLaneChange = false;
    }

    
    if (!checkedForLaneChange && s + LaneChangeDistance > currLaneLength && 
        routingGraph.get_lane_successors(currKey).empty())
    {
        checkedForLaneChange = true;
        tOffset = 0;
        // Change lane if current Lane ends within LaneChangeDistance
        const auto& section = odrMap.id_to_road.at(currKey.road_id).get_lanesection(currKey.lanesection_s0);
        auto neighborLanes = section.get_sorted_driving_lanes(currKey.lane_id > 0 ? 1 : -1);
        for (auto& neighbor : neighborLanes)
        {
            if (neighbor.id == currKey.lane_id) continue;
            if (!routingGraph.get_lane_successors(neighbor.key).empty())
            {
                double sOnRefLine = currKey.lane_id > 0 ? currLaneLength - s : s;
                double tBase = section.id_to_lane.at(currKey.lane_id).outer_border.get(sOnRefLine + currKey.lanesection_s0);
                double tTarget = section.id_to_lane.at(neighbor.id).outer_border.get(sOnRefLine + currKey.lanesection_s0);
                tOffset = tBase - tTarget;

                currKey = neighbor.key;
                break;
            }
        }
    }
    
    // Update transform
    const auto& road = odrMap.id_to_road.at(currKey.road_id);
    const auto& section = road.get_lanesection(currKey.lanesection_s0);
    bool reversedTraverse = currKey.lane_id > 0;
    double sOnRefLine = reversedTraverse ? currLaneLength - s : s;
    double tInner = section.id_to_lane.at(currKey.lane_id).inner_border.get(sOnRefLine + currKey.lanesection_s0);
    double tOuter = section.id_to_lane.at(currKey.lane_id).outer_border.get(sOnRefLine + currKey.lanesection_s0);
    double tCenter = (tInner + tOuter) / 2;

    auto pos3 = road.get_xyz(sOnRefLine + currKey.lanesection_s0, tCenter + tOffset, 0);

    double gradFromLane = section.id_to_lane.at(currKey.lane_id).outer_border.get_grad(sOnRefLine + currKey.lanesection_s0);
    double angleFromLane = 180 / M_PI * std::atan2(gradFromLane, 1);
    auto angleFromlaneChange = 180 / M_PI * std::atan2(-laneChangeRate, 1);

    auto gradFromRefLine = road.ref_line.get_grad_xy(sOnRefLine + currKey.lanesection_s0);
    auto angleFromRefLine = 180 / M_PI * std::atan2(gradFromRefLine[1], gradFromRefLine[0]);

    auto angle = angleFromRefLine + angleFromLane + angleFromlaneChange;
    if (reversedTraverse) angle += 180;

    auto roadElevation = road.ref_line.elevation_profile.get(sOnRefLine + currKey.lanesection_s0);

    graphics->setPos(QPointF(pos3[0], pos3[1]));
    graphics->setRotation(angle);
    graphics->setZValue(roadElevation + 2.5);
    graphics->show();
    return true;
}

void Vehicle::updateCurrKeyLength(const odr::OpenDriveMap& odrMap)
{
    auto idAndRoad = odrMap.id_to_road.find(currKey.road_id);
    if (idAndRoad == odrMap.id_to_road.end())
    {
        currLaneLength = 0;
        return;
    }

    try
    {
        const auto& laneSection = idAndRoad->second.get_lanesection(currKey.lanesection_s0);
        currLaneLength = idAndRoad->second.get_lanesection_length(laneSection);
    }
    catch (std::runtime_error)
    {
        currLaneLength = 0;
    }
}