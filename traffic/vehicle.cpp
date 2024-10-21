#include "vehicle.h"
#include "OpenDriveMap.h"
#include "constants.h"

#include <qgraphicsscene.h>
#include <math.h>
#include "spdlog/spdlog.h"
#include "id_generator.h"

extern QGraphicsScene* g_scene;

Vehicle::Vehicle(odr::LaneKey initialLane, double initialLocalS,
    odr::LaneKey destLane, double destS) :
    s(initialLocalS), firstStep(true),
    currLaneLength(0), tOffset(0), laneChangeDueS(0),
    ID(IDGenerator::ForVehicle()->GenerateID(this)), DestLane(destLane), DestS(destS)
{
    navigation = { initialLane };
    graphics = new QGraphicsRectItem(QRectF(-2.3, -0.9, 4.6, 1.8));
    graphics->setPen(Qt::NoPen);
    auto randColor = static_cast<Qt::GlobalColor>(rand() % static_cast<int>(Qt::GlobalColor::transparent));
    graphics->setBrush(QBrush(randColor, Qt::SolidPattern));
    g_scene->addItem(graphics);
    graphics->hide();

    leaderVisual = new QGraphicsLineItem();
    g_scene->addItem(leaderVisual);
    leaderVisual->hide();
}

void Vehicle::Clear()
{
    g_scene->removeItem(graphics);
    g_scene->removeItem(leaderVisual);
    IDGenerator::ForVehicle()->FreeID(ID);
}

bool Vehicle::Step(double dt, const odr::OpenDriveMap& odrMap, const odr::RoutingGraph& routingGraph,
    const std::unordered_map<odr::LaneKey, std::map<double, std::shared_ptr<Vehicle>>>& vehiclesOnLane)
{
    if (firstStep)
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
        firstStep = false;
    }

    auto leader = GetLeader(odrMap, vehiclesOnLane);
    if (leader != nullptr)
    {
        auto myTip = TipPos();
        auto leaderTail = leader->TailPos();
        leaderVisual->setLine(myTip[0], myTip[1], leaderTail[0], leaderTail[1]);
        leaderVisual->show();
    }
    else
    {
        leaderVisual->hide();
    }

    double laneChangeRate = 0;
    if (s < laneChangeDueS)
    {
        double remainingS = laneChangeDueS - s;
        laneChangeRate = tOffset / remainingS * 1.5;
        tOffset -= laneChangeRate * dt * DefaultVelocity;
        if (std::abs(tOffset) < 0.3)
        {
            // mark lane change as complete
            lcFrom.reset();
        }
    }

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
        lcFrom.emplace(navigation.front());
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

    position = road.get_xyz(sOnRefLine + currKey.lanesection_s0, tCenter + tOffset, 0);

    double gradFromLane = section.id_to_lane.at(currKey.lane_id).outer_border.get_grad(sOnRefLine + currKey.lanesection_s0);
    double angleFromLane = 180 / M_PI * std::atan2(gradFromLane, 1);
    auto angleFromlaneChange = 180 / M_PI * std::atan2(-laneChangeRate, 1);

    auto gradFromRefLine = road.ref_line.get_grad_xy(sOnRefLine + currKey.lanesection_s0);
    auto angleFromRefLine = 180 / M_PI * std::atan2(gradFromRefLine[1], gradFromRefLine[0]);

    heading = angleFromRefLine + angleFromLane + angleFromlaneChange;
    if (reversedTraverse) heading += 180;

    graphics->setPos(QPointF(position[0], position[1]));
    graphics->setRotation(heading);
    auto surfaceZ = road.ref_line.elevation_profile.get_max(
        sOnRefLine + currKey.lanesection_s0 - RoadRunner::GraphicsDivision, 
        sOnRefLine + currKey.lanesection_s0 + RoadRunner::GraphicsDivision);
    graphics->setZValue(surfaceZ + 0.01);
    graphics->show();
    return true;
}

std::vector<odr::LaneKey> Vehicle::OccupyingLanes() const
{
    std::vector<odr::LaneKey> rtn = { navigation.front() };
    if (lcFrom.has_value())
    {
        rtn.push_back(lcFrom.value());
    }
    return rtn;
}

odr::Vec3D Vehicle::TipPos() const
{
    auto offset = odr::mut(2.3, odr::Vec3D{ std::cos(heading * M_PI / 180), std::sin(heading * M_PI / 180), 0 });
    return odr::add(offset, position);
}

odr::Vec3D Vehicle::TailPos() const
{
    auto offset = odr::mut(-2.3, odr::Vec3D{ std::cos(heading * M_PI / 180), std::sin(heading * M_PI / 180), 0 });
    return odr::add(offset, position);
}

double Vehicle::CurrS() const
{
    return s;
}

std::shared_ptr<Vehicle> Vehicle::GetLeader(const odr::OpenDriveMap& map,
    const std::unordered_map<odr::LaneKey, std::map<double, std::shared_ptr<Vehicle>>>& vehiclesOnLane,
    double lookforward) const
{
    // curr lane:
    // if lane changing, consider leader on both lanes
    std::shared_ptr<Vehicle> rtn;
    for (auto laneKey : OccupyingLanes())
    {
        auto& orderedOnLane = vehiclesOnLane.at(laneKey);
        auto it = orderedOnLane.upper_bound(s);
        for (; it != orderedOnLane.end(); ++it)
        {
            if (it->second->ID == ID)
            {
                continue;
            }
            rtn = it->second;
            break;
        }
    }

    if (lcFrom.has_value())
    {
        auto& orderedOnLane = vehiclesOnLane.at(lcFrom.value());
        auto it = orderedOnLane.upper_bound(s);
        for (; it != orderedOnLane.end(); ++it)
        {
            if (it->second->ID == ID)
            {
                continue;
            }
            if (rtn == nullptr || it->second->CurrS() < rtn->CurrS())
            {
                rtn = it->second;
            }
            break;
        }
    }

    if (rtn != nullptr)
        return rtn;

    // lanes ahead navigation
    double goingForwardS = currLaneLength;
    for (int i = 1; i < navigation.size(); ++i)
    {
        if (goingForwardS > lookforward)
        {
            return rtn;
        }
        auto& orderedOnLane = vehiclesOnLane.at(navigation[i]);
        if (orderedOnLane.begin() != orderedOnLane.end())
        {
            return orderedOnLane.begin()->second;
        }
        goingForwardS += map.id_to_road.at(navigation[i].road_id).get_lanesection_length(navigation[i].lanesection_s0);
    }
    return rtn;
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