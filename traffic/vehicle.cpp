#include "vehicle.h"
#include "OpenDriveMap.h"
#include "constants.h"

#include <qgraphicsscene.h>
#include <math.h>
#include "spdlog/spdlog.h"

extern QGraphicsScene* g_scene;

Vehicle::Vehicle(odr::LaneKey initialLane, double initialLocalS,
    odr::LaneKey destLane, double destS) :
    s(initialLocalS), odoMeter(0),
    currLaneLength(0), tOffset(0), laneChangeDueS(0),
    DestLane(destLane), DestS(destS)
{
    navigation = { initialLane };
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
        updateNavigation(odrMap, routingGraph);
        if (navigation.empty())
        {
            // No path or dest already reached
            return false;
        }

        const auto currKey = navigation.front();
        const auto& road = odrMap.id_to_road.at(currKey.road_id);
        const auto& section = road.get_lanesection(currKey.lanesection_s0);
        currLaneLength = road.get_lanesection_length(section);
    }

    double laneChangeRate = 0;
    if (s < laneChangeDueS)
    {
        double remainingS = laneChangeDueS - s;
        laneChangeRate = tOffset / remainingS * 1.5;
        tOffset -= laneChangeRate * dt * DefaultVelocity;
    }

    odoMeter += dt * DefaultVelocity;
    s += dt * DefaultVelocity;

    if (std::equal_to<odr::LaneKey>{}(navigation.front(), DestLane) &&
        s < DestS != DestLane.lane_id < 0)
    {
        // already past destination s
        return false;
    }
    
    if (navigation.size() >= 2 &&
        navigation[0].road_id == navigation[1].road_id &&
        std::abs(navigation[0].lane_id - navigation[1].lane_id) == 1)
    {
        // Next move is lane switch
        auto currKey = navigation.front();
        double sOnRefLine = currKey.lane_id > 0 ? currLaneLength - s : s;
        const auto& section = odrMap.id_to_road.at(currKey.road_id).get_lanesection(currKey.lanesection_s0);
        double tBase = section.id_to_lane.at(currKey.lane_id).outer_border.get(sOnRefLine + currKey.lanesection_s0);
        double tTarget = section.id_to_lane.at(navigation[1].lane_id).outer_border.get(sOnRefLine + currKey.lanesection_s0);
        tOffset = tBase - tTarget;
        navigation.erase(navigation.begin());
        if (navigation.empty())
        {
            return false;
        }
        // TODO: Complete consecutive lane changes in time
        laneChangeDueS = std::min(currLaneLength / 2, MaxSwitchLaneDistance);
    }
    else
    {
        if (s > currLaneLength)
        {
            navigation.erase(navigation.begin());
            if (navigation.empty())
            {
                return false;
            }
            s -= currLaneLength;

            const auto currKey = navigation.front();
            const auto& road = odrMap.id_to_road.at(currKey.road_id);
            const auto& section = road.get_lanesection(currKey.lanesection_s0);
            currLaneLength = road.get_lanesection_length(section);
        }
    }
    
    // Update transform
    const auto currKey = navigation.front();

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
    auto surfaceZ = road.ref_line.elevation_profile.get_max(
        sOnRefLine + currKey.lanesection_s0 - RoadRunner::GraphicsDivision, 
        sOnRefLine + currKey.lanesection_s0 + RoadRunner::GraphicsDivision);
    graphics->setZValue(surfaceZ + 0.01);
    graphics->show();
    return true;
}

void Vehicle::updateNavigation(const odr::OpenDriveMap& odrMap, const odr::RoutingGraph& routingGraph)
{
    if (navigation.empty())
    {
        return;
    }

    bool dueRecal = false;
    if (!std::equal_to<odr::LaneKey>{}(navigation.back(), DestLane))
    {
        dueRecal = true;
    }
    // TODO: check if path is still valid in odrMap, in case there's an edit

    if (dueRecal)
    {
        if (std::equal_to<odr::LaneKey>{}(navigation.front(), DestLane))
        {
            if (s < DestS == DestLane.lane_id < 0)
            {
                navigation = { DestLane };
            }
            else
            {
                navigation.clear();
            }
        }
        else
        {
            navigation = routingGraph.shortest_path(navigation.front(), DestLane);

            spdlog::info("<shortest_path> size = {}", navigation.size());
            for (auto segment : navigation)
            {
                spdlog::info("  Road{} lane{}", segment.road_id, segment.lane_id);
            }
        }
    }
}