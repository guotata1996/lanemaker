#include "vehicle.h"
#include "OpenDriveMap.h"
#include "constants.h"

#include <qgraphicsscene.h>
#include <math.h>
#include "spdlog/spdlog.h"
#include "id_generator.h"

extern QGraphicsScene* g_scene;

Vehicle::Vehicle(odr::LaneKey initialLane, double initialLocalS, odr::LaneKey destLane, double destS, double maxV) :
    AS(initialLocalS), ALane(initialLane), BS(destS), BLane(destLane),
    currLaneLength(0), tOffset(0), laneChangeDueS(0), velocity(0), MaxV(maxV), firstStep(true),
    ID(IDGenerator::ForVehicle()->GenerateID(this)), goalIndex(false)
{
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

bool Vehicle::GotoNextGoal(const odr::OpenDriveMap& odrMap, const odr::RoutingGraph& routingGraph)
{
    goalIndex = !goalIndex;
    updateNavigation(odrMap, routingGraph);
    if (ID == "12")
        spdlog::info("GotoNextG");
    s = sourceS();
    tOffset = 0;
    laneChangeDueS = 0;

    const auto currKey = sourceLane();
    const auto& road = odrMap.id_to_road.at(currKey.road_id);
    const auto& section = road.get_lanesection(currKey.lanesection_s0);
    currLaneLength = road.get_lanesection_length(section);

    return !navigation.empty();
}

void Vehicle::Clear()
{
    g_scene->removeItem(graphics);
    g_scene->removeItem(leaderVisual);
    IDGenerator::ForVehicle()->FreeID(ID);
}

bool Vehicle::PlanStep(double dt, const odr::OpenDriveMap& odrMap,
    const std::unordered_map<odr::LaneKey, std::map<double, std::shared_ptr<Vehicle>>>& vehiclesOnLane)
{
    double recordV = velocity, recordS = s;

    double leaderDistance;
    auto leader = GetLeader(odrMap, vehiclesOnLane, leaderDistance);
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
    new_velocity = vFromGibbs(dt, leader, leaderDistance);
    new_s = s + dt * new_velocity;

    if (std::equal_to<odr::LaneKey>{}(navigation.front(), destLane()) &&
        s < destS() && new_s >= destS())
    {
        // Past destination s
        assert(recordV == velocity && recordS == s);
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
        assert(!navigation.empty());

        // TODO: Complete consecutive lane changes in time
        laneChangeDueS = std::min((currLaneLength + s) / 2, MaxSwitchLaneDistance + s);
    }
    else
    {
        if (new_s > currLaneLength)
        {
            navigation.erase(navigation.begin());
            assert(!navigation.empty());

            new_s = 0;

            const auto currKey = navigation.front();
            const auto& road = odrMap.id_to_road.at(currKey.road_id);
            const auto& section = road.get_lanesection(currKey.lanesection_s0);
            currLaneLength = road.get_lanesection_length(section);
        }
    }
    
    assert(recordV == velocity && recordS == s);
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
    double& outDistance, double lookforward) const
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
            if (rtn == nullptr || it->second->s < rtn->s)
            {
                rtn = it->second;
            }
            break;
        }
    }

    if (rtn != nullptr)
    {
        outDistance = rtn->s - s;
        assert(outDistance > 0);
        return outDistance < lookforward ? rtn : nullptr;
    }

    // lanes ahead navigation
    outDistance = currLaneLength - s;
    assert(outDistance >= 0);
    for (int i = 1; i < navigation.size(); ++i)
    {
        if (outDistance > lookforward)
        {
            return rtn;
        }
        if (vehiclesOnLane.find(navigation[i]) != vehiclesOnLane.end())
        {
            auto& orderedOnLane = vehiclesOnLane.at(navigation[i]);
            if (orderedOnLane.begin() != orderedOnLane.end())
            {
                outDistance += orderedOnLane.begin()->second->s;
                assert(outDistance >= 0);
                rtn = orderedOnLane.begin()->second;
                return outDistance < lookforward ? rtn : nullptr;
            }
        }

        outDistance += map.id_to_road.at(navigation[i].road_id).get_lanesection_length(navigation[i].lanesection_s0);
    }
    return rtn;
}

double Vehicle::vFromGibbs(double dt, std::shared_ptr<Vehicle> leader, double distance) const
{
    // Gipps model
    const double tau = 1.5; // reaction time
    const double a = 3; // max acc
    const double b = -8;  // max dcc
    const double s0 = 4;  // static gap
    const double li = 4.6;

    double vOut = velocity + 2.5 * a * dt * (1 - velocity / MaxV) *
        std::sqrt(0.025 + velocity / MaxV);

    if (leader != nullptr)
    {
        double underSqr = std::pow(b * dt, 2) - b *
            (-velocity * tau - std::pow(leader->velocity, 2) / b - 2 * li - s0 + 2 * distance);
        if (underSqr < 0)
        {
            // gonna collide, hard stop
            vOut = 0;
        }
        else
        {
            double vFollow = -b * dt + std::sqrt(underSqr);
            vOut = std::min(vOut, vFollow);
        }
    }

    assert(vOut >= 0);
    return vOut;
}

void Vehicle::updateNavigation(const odr::OpenDriveMap& odrMap, const odr::RoutingGraph& routingGraph)
{
    const auto Source = sourceLane();
    const auto Dest = destLane();

    if (Source.road_id == Dest.road_id && Source.lanesection_s0 == Dest.lanesection_s0
        && Source.lane_id * Dest.lane_id > 0)
    {
        if (sourceS() < destS())
        {
            navigation = { Dest };
        }
        else
        {
            navigation.clear();
            
            for (auto second : routingGraph.get_lane_successors(Source))
            {
                navigation = routingGraph.shortest_path(second, Dest);
                if (!navigation.empty())
                {
                    navigation.insert(navigation.begin(), Source);
                    break;
                }
            }
        }
    }
    else
    {
        navigation = routingGraph.shortest_path(Source, Dest);
    }

    if (navigation.empty())
    {
        spdlog::info("No route find form {} @{} to {} @{}", Source.to_string(), s, 
            Dest.to_string(), destS());
    }
}

void Vehicle::MakeStep(double dt, const odr::OpenDriveMap& map)
{
    velocity = new_velocity;
    s = new_s;

    double laneChangeRate = 0;
    if (s < laneChangeDueS - 0.1)
    {
        double remainingS = laneChangeDueS - s;
        laneChangeRate = tOffset / remainingS * 1.5;
        tOffset -= laneChangeRate * dt * velocity;
    }
    else
    {
        lcFrom.reset();
    }
    if (std::abs(tOffset) < 0.3)
    {
        // mark lane change as complete
        lcFrom.reset();
    }

    // Update transform
    const auto currKey = navigation.front();

    const auto& road = map.id_to_road.at(currKey.road_id);
    const auto& section = road.get_lanesection(currKey.lanesection_s0);
    bool reversedTraverse = currKey.lane_id > 0;
    double sOnRefLine = reversedTraverse ? currLaneLength - s : s;
    double tInner = section.id_to_lane.at(currKey.lane_id).inner_border.get(sOnRefLine + currKey.lanesection_s0);
    double tOuter = section.id_to_lane.at(currKey.lane_id).outer_border.get(sOnRefLine + currKey.lanesection_s0);
    double tCenter = (tInner + tOuter) / 2;

    auto newPos = road.get_xyz(sOnRefLine + currKey.lanesection_s0, tCenter + tOffset, 0);
    if (firstStep)
    {
        firstStep = false;
    }
    else
    {
        if (ID == "12")
        {
            if (odr::euclDistance(position, newPos) < MaxV * dt * 2)
            {
                spdlog::info("PASS");
            }
            else
            {
                spdlog::info("Fail");
            }
        }
        //assert(odr::euclDistance(position, newPos) < MaxV * dt * 2);
    }
    position = newPos;

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
}

odr::LaneKey Vehicle::sourceLane() const
{
    return goalIndex ? ALane : BLane;
}

odr::LaneKey Vehicle::destLane() const
{
    return goalIndex ? BLane : ALane;
}

double Vehicle::sourceS() const
{
    return goalIndex ? AS : BS;
}

double Vehicle::destS() const
{
    return goalIndex ? BS : AS;
}