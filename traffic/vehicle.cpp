#include "vehicle.h"
#include "OpenDriveMap.h"
#include "constants.h"
#include "id_generator.h"

#include <qgraphicsscene.h>
#include <math.h>
#include <sstream>
#include "spdlog/spdlog.h"


extern QGraphicsScene* g_scene;
const std::string NowDebugging = "";

VehicleGraphics::VehicleGraphics(QRectF r, std::weak_ptr<Vehicle> v) : QGraphicsRectItem(r), vehicle(v) {};

Vehicle::Vehicle(odr::LaneKey initialLane, double initialLocalS, odr::LaneKey destLane, double destS, double maxV) :
    AS(initialLocalS), ALane(initialLane), BS(destS), BLane(destLane),
    currLaneLength(0), tOffset(0), laneChangeDueS(0), velocity(0), MaxV(maxV), step(0),
    ID(IDGenerator::ForVehicle()->GenerateID(this)), goalIndex(false)
{
}

void Vehicle::InitGraphics()
{
    graphics = new VehicleGraphics(QRectF(-2.3, -0.9, 4.6, 1.8), shared_from_this());
    graphics->setPen(Qt::NoPen);
    auto randColor = static_cast<Qt::GlobalColor>(rand() % static_cast<int>(Qt::GlobalColor::transparent));
    graphics->setBrush(QBrush(randColor, Qt::SolidPattern));
    graphics->hide();
    g_scene->addItem(graphics);

    leaderVisual = new QGraphicsLineItem();
    g_scene->addItem(leaderVisual);
    leaderVisual->hide();
}

bool Vehicle::GotoNextGoal(const odr::OpenDriveMap& odrMap, const odr::RoutingGraph& routingGraph)
{
    assert(std::abs(tOffset) < LCCompleteThreshold);
    goalIndex = !goalIndex;
    updateNavigation(odrMap, routingGraph);
    s = sourceS();
    laneChangeDueS = 0;

    const auto currKey = sourceLane();
    currLaneLength = odrMap.get_lanekey_length(currKey);

    if (ID == NowDebugging)
    {
        spdlog::info("==== begin navigation ====");
        for (auto n : navigation)
        {
            spdlog::info(n.to_string());
        }
        spdlog::info("====  end  navigation ====");
    }

    return !navigation.empty();
}

void Vehicle::Clear()
{
    g_scene->removeItem(graphics);
    g_scene->removeItem(leaderVisual);
    IDGenerator::ForVehicle()->FreeID(ID);
}

bool Vehicle::PlanStep(double dt, const odr::OpenDriveMap& odrMap,
    const std::unordered_map<odr::LaneKey, 
    std::map<double, std::shared_ptr<Vehicle>>>& vehiclesOnLane,
    const std::map<odr::LaneKey, std::vector<std::pair<odr::LaneKey, double>>>& overlapZones)
{
    double recordV = velocity, recordS = s;

    double leaderDistance;
    auto leader = GetLeader(odrMap, vehiclesOnLane, overlapZones, leaderDistance);
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
        s <= destS() && new_s > destS())
    {
        // Past destination s
        assert(recordV == velocity && recordS == s);
        return false;
    }

    if (navigation.size() >= 2 &&
        navigation[0].road_id == navigation[1].road_id &&
        navigation[0].lanesection_s0 == navigation[1].lanesection_s0 &&
        std::abs(navigation[0].lane_id - navigation[1].lane_id) == 1 &&
        !lcFrom.has_value())
    {
        // Next move is lane switch
        auto currKey = navigation.front();
        double sOnRefLine = currKey.lane_id > 0 ? currLaneLength - s : s;
        const auto& section = odrMap.id_to_road.at(currKey.road_id).get_lanesection(currKey.lanesection_s0);
        double tBase = section.id_to_lane.at(currKey.lane_id).outer_border.get(sOnRefLine + currKey.lanesection_s0);
        double tTarget = section.id_to_lane.at(navigation[1].lane_id).outer_border.get(sOnRefLine + currKey.lanesection_s0);
        tOffset += tBase - tTarget;
        if (ID == NowDebugging)
        {
            spdlog::info("  Switching lane from {} to {} brings tOffset to {}", currKey.to_string(), navigation[1].to_string(), tOffset);
        }
        lcFrom.emplace(navigation.front());
        navigation.erase(navigation.begin());
        assert(!navigation.empty());

        if (std::equal_to<odr::LaneKey>{}(navigation.front(), destLane()) &&
            s <= destS() && new_s > destS())
        {
            // Past destination s
            assert(recordV == velocity && recordS == s);
            return false;
        }

        int consecutiveLC = 1;
        for (auto next : navigation)
        {
            if (next.road_id == lcFrom->road_id &&
                next.lanesection_s0 == lcFrom->lanesection_s0 &&
                (next.lane_id > 0) == (lcFrom->lane_id > 0))
            {
                consecutiveLC++;
            }
            else
            {
                break;
            }
        }
        if (ID == NowDebugging && consecutiveLC > 1)
        {
            spdlog::info("Consecutive LC: {}", consecutiveLC);
        }
        auto lastLaneChangeDueS = navigation.front().road_id == destLane().road_id &&
            navigation.front().lanesection_s0 == destLane().lanesection_s0 &&
            navigation.front().lane_id > 0 == destLane().lane_id > 0 ? destS() : currLaneLength;

        laneChangeDueS = std::min(s + (lastLaneChangeDueS - s) / consecutiveLC, MaxSwitchLaneDistance + s);
    }
    else
    {
        if (new_s > currLaneLength)
        {
            assert(std::abs(tOffset) < LCCompleteThreshold);
            auto erasedNavigation = navigation.front();
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
    const std::map<odr::LaneKey, std::vector<std::pair<odr::LaneKey, double>>>& overlapZones,
    double& outDistance, double lookforward) const
{
    // curr lane:
    // if lane changing, consider leader on both lanes
    std::shared_ptr<Vehicle> rtn;
    for (auto laneKey : OccupyingLanes())
    {
        if (vehiclesOnLane.find(laneKey) == vehiclesOnLane.end())
        {
            continue;
        }

        auto& orderedOnLane = vehiclesOnLane.at(laneKey);
        for (auto it = orderedOnLane.upper_bound(s); it != orderedOnLane.end(); ++it)
        {
            if (rtn == nullptr || it->second->s < rtn->s)
            {
                rtn = it->second;
                outDistance = rtn->s - s;
            }
            break;
        }
    }

    // if curr lane has overlapZone, consider those on overlap lanes
    const auto& directlyOn = navigation.front();
    if (overlapZones.find(directlyOn) != overlapZones.end())
    {
        for (const auto& overlap_and_lane : overlapZones.at(directlyOn))
        {
            double overlapLength = overlap_and_lane.second;
            if (overlapLength > 0 && s >= overlapLength ||
                overlapLength < 0 && s <= currLaneLength + overlapLength)
            {
                continue;
            }

            auto overlapLane = overlap_and_lane.first;
            if (vehiclesOnLane.find(overlapLane) == vehiclesOnLane.end())
            {
                continue;
            }

            double equalSOnOther = overlapLength > 0 ? s : map.get_lanekey_length(overlapLane) - (currLaneLength - s);
            auto& orderedOnLane = vehiclesOnLane.at(overlapLane);

            for (auto it = orderedOnLane.upper_bound(equalSOnOther); it != orderedOnLane.end(); ++it)
            {
                if (overlapLength > 0 && it->second->s > overlapLength)
                {
                    // ignore those outside overlapZone
                    break;
                }
                double distBetween = overlapLength > 0 ? it->second->s - s :
                    (currLaneLength - s) - (map.get_lanekey_length(overlapLane) - it->second->s);

                if (rtn == nullptr || distBetween < outDistance)
                {
                    rtn = it->second;
                    outDistance = distBetween;
                }
                break;
            }
        }
        
        
    }

    if (rtn != nullptr)
    {
        assert(outDistance > 0);
        return outDistance < lookforward ? rtn : nullptr;
    }

    // lanes ahead navigation
    // TODO: consider overlap zone
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
            vOut = distance < li ? 0 : std::max(leader->velocity + b * dt, 0.0);
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
            navigation = { Source, Dest };
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

    if (ID == NowDebugging)
    {
        spdlog::info("  Update nav: {}", navigation.size());
    }
}

void Vehicle::MakeStep(double dt, const odr::OpenDriveMap& map)
{
    velocity = new_velocity;
    s = new_s;

    double laneChangeRate = 0;
    if (s < laneChangeDueS - 0.1 && lcFrom.has_value())
    {
        double remainingS = laneChangeDueS - s;
        laneChangeRate = tOffset / remainingS * 1.5;
        tOffset -= laneChangeRate * dt * velocity;
    }
    else
    {
        lcFrom.reset();
    }
    if (std::abs(tOffset) < LCCompleteThreshold)
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
    if (ID == NowDebugging)
    {
        spdlog::info("[{}] Lane {} s={} tOffset={} t={} | G= {} @{} Nav:{}", step, currKey.to_string(), s, tOffset, tCenter + tOffset,
            destLane().to_string(), destS(), navigation.size());
    }

    step++;
    position = newPos;

    double gradFromLane = section.id_to_lane.at(currKey.lane_id).outer_border.get_grad(sOnRefLine + currKey.lanesection_s0);
    double angleFromLane = 180 / M_PI * std::atan2(gradFromLane, 1);
    auto angleFromlaneChange = 180 / M_PI * std::atan2(-laneChangeRate * (reversedTraverse ? -1 : 1), 1);

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

std::string Vehicle::Log()
{
    std::stringstream ss;
    ss << "Vehicle ID=" << ID << "  s=" << s << "  v=" << velocity << '\n';
    ss << "From " << sourceLane().to_string() << "@" << sourceS() <<
        " To " << destLane().to_string() << "@" << destS() << '\n';
    return ss.str();
}