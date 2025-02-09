#include "vehicle.h"
#include "OpenDriveMap.h"
#include "constants.h"
#include "id_generator.h"
#include "map_view_gl.h"
#include "spatial_indexer.h"

#include <math.h>
#include <sstream>
#include "spdlog/spdlog.h"


const std::string NowDebugging = "";

QVector3D Vehicle::DimensionLWH = QVector3D(4.6, 1.8, 1.6);

Vehicle::Vehicle(odr::LaneKey initialLane, double initialLocalS, odr::LaneKey destLane, double destS, double maxV) :
    AS(initialLocalS), ALane(initialLane), BS(destS), BLane(destLane),
    currLaneLength(0), tOffset(0), laneChangeDueS(0), velocity(0), MaxV(maxV), stepInJunction(0),
    ID(IDGenerator::ForType(IDType::Vehicle)->GenerateID(this)), goalIndex(false)
{
}

void Vehicle::InitGraphics()
{
    graphics.emplace(std::stoi(ID), RoadRunner::InstanceData::GetRandom());
}

bool Vehicle::GotoNextGoal(const odr::OpenDriveMap& odrMap, const odr::RoutingGraph& routingGraph,
    const std::unordered_map<odr::LaneKey, int>& nVehiclesOnLane)
{
    if (stepInJunction > DestroyIfInJunction)
    {
        // Abort if stuck
        return false;
    }
    assert(std::abs(tOffset) < LCCompleteThreshold);
    goalIndex = !goalIndex;
    updateNavigation(odrMap, routingGraph, nVehiclesOnLane);
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
    graphics.reset();
    RoadRunner::SpatialIndexerDynamic::Instance()->UnIndex(std::stoi(ID));
    IDGenerator::ForType(IDType::Vehicle)->FreeID(ID);
}

void Vehicle::EnableRouteVisual(bool enabled, const odr::OpenDriveMap& odrMap)
{
    routeVisual.Clear();

    if (enabled)
    {
        for (int i = 0; i != navigation.size(); ++i)
        {
            double sBeginOnLane = i == 0 ? s : 0;
            double sEndOnLane = i == navigation.size() - 1 ? destS() : odrMap.get_lanekey_length(navigation[i]);
            assert(sBeginOnLane < sEndOnLane);

            auto localLine = odrMap.id_to_road.at(navigation[i].road_id).get_lane_center_line(
                navigation[i], sBeginOnLane, sEndOnLane, 1.0);

            odr::Line3D liftedVisual;
            for (const auto& p: localLine)
            {
                liftedVisual.emplace_back(odr::add(p, odr::Vec3D{ 0, 0, DimensionLWH.z() / 2}));
            }
            routeVisual.AddLine(liftedVisual, 0.3, Qt::green);
        }
        if (!leaderLine.empty())
        {
            routeVisual.AddLine(leaderLine, 0.3, Qt::black);
        }
    }
}

bool Vehicle::PlanStep(double dt, const odr::OpenDriveMap& odrMap,
    const std::unordered_map<odr::LaneKey, 
    std::map<double, std::shared_ptr<Vehicle>>>& vehiclesOnLane,
    const std::map<odr::LaneKey, std::vector<std::pair<odr::LaneKey, double>>>& overlapZones,
    const std::unordered_map<odr::LaneKey, bool>& signalStates)
{
    double recordV = velocity, recordS = s;

    double leaderDistance;
    auto leader = GetLeader(odrMap, vehiclesOnLane, overlapZones, signalStates, leaderDistance);
    leaderLine.clear();
    if (leader != nullptr)
    {
        auto myTip = TipPos();
        auto leaderTail = leader->TailPos();
        if (odr::euclDistance(myTip, leaderTail) > 1e-3)
        {
            const auto lift = odr::Vec3D{ 0, 0, DimensionLWH.z()};
            leaderLine = { odr::add(myTip, lift), odr::add(leaderTail, lift) };
        }
    }
    new_velocity = vFromGibbs(dt, leader, leaderDistance);
    new_s = s + dt * new_velocity;

    if (signalStates.find(navigation.front()) != signalStates.end())
    {
        stepInJunction++;
        if (stepInJunction > DestroyIfInJunction)
        {
            return false;
        }
    }
    else
    {
        stepInJunction = 0;
    }
    
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
            if (navigation.empty())
            {
                spdlog::warn("Vehicle {} fails to reach goal", ID);
                return false;
            }

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
    if (lcFrom.has_value() && std::abs(tOffset) > 0.6)
    {
        rtn.push_back(lcFrom.value());
    }
    return rtn;
}

odr::Vec3D Vehicle::TipPos() const
{
    auto offset = odr::mut(DimensionLWH.x() / 2.0, odr::Vec3D{ std::cos(heading), std::sin(heading), 0 });
    return odr::add(offset, position);
}

odr::Vec3D Vehicle::TailPos() const
{
    auto offset = odr::mut(-DimensionLWH.x() / 2.0, odr::Vec3D{ std::cos(heading), std::sin(heading), 0 });
    return odr::add(offset, position);
}

double Vehicle::S() const
{
    return s;
}

double Vehicle::V() const
{
    return velocity;
}

std::shared_ptr<Vehicle> Vehicle::GetLeaderInOverlapZone(
    odr::LaneKey lane, double s0,
    const odr::OpenDriveMap& map,
    const std::unordered_map<odr::LaneKey, std::map<double, std::shared_ptr<Vehicle>>>& vehiclesOnLane,
    const std::map<odr::LaneKey, std::vector<std::pair<odr::LaneKey, double>>>& overlapZones,
    double& outDistance, double lookforward)
{
    std::shared_ptr<Vehicle> rtn;
    outDistance = 1e9;
    if (overlapZones.find(lane) != overlapZones.end())
    {
        double currLaneLength = map.get_lanekey_length(lane);
        for (const auto& overlap_and_lane : overlapZones.at(lane))
        {
            double overlapLength = overlap_and_lane.second;
            if (overlapLength > 0 && s0 >= overlapLength)
            {
                continue;
            }

            auto overlapLane = overlap_and_lane.first;
            if (vehiclesOnLane.find(overlapLane) == vehiclesOnLane.end())
            {
                continue;
            }

            double othersLaneLength = map.get_lanekey_length(overlapLane);
            double equalSOnOther = overlapLength > 0 ? s0 : othersLaneLength - (currLaneLength - s0);
            auto& orderedOnLane = vehiclesOnLane.at(overlapLane);

            for (auto it = orderedOnLane.upper_bound(equalSOnOther); it != orderedOnLane.end(); ++it)
            {
                if (overlapLength > 0 && it->second->s > overlapLength ||
                    overlapLength < 0 && it->second->s < othersLaneLength + overlapLength)
                {
                    // ignore those outside overlapZone
                    continue;
                }
                double distBetween = overlapLength > 0 ? it->second->s - s0 :
                    (currLaneLength - s0) - (othersLaneLength - it->second->s);

                if (rtn == nullptr || distBetween < outDistance)
                {
                    rtn = it->second;
                    outDistance = distBetween;
                }
                break;
            }
        }
    }
    return rtn;
}

std::shared_ptr<const Vehicle> Vehicle::GetLeader(const odr::OpenDriveMap& map,
    const std::unordered_map<odr::LaneKey, std::map<double, std::shared_ptr<Vehicle>>>& vehiclesOnLane,
    const std::map<odr::LaneKey, std::vector<std::pair<odr::LaneKey, double>>>& overlapZones,
    const std::unordered_map<odr::LaneKey, bool>& signalStates,
    double& outDistance, double lookforward) const
{
    // curr lane:
    // if lane changing, consider leader on both lanes
    std::shared_ptr<const Vehicle> rtn;
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
    double onOverlapZoneDistance;
    auto onOverlapZone = GetLeaderInOverlapZone(navigation.front(), s, map, vehiclesOnLane, overlapZones, onOverlapZoneDistance, lookforward);
    if (onOverlapZone != nullptr && 
        (rtn == nullptr || onOverlapZoneDistance < outDistance))
    {
        rtn = onOverlapZone;
        outDistance = onOverlapZoneDistance;
    }

    if (rtn != nullptr)
    {
        assert(rtn != shared_from_this());
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
        double distanceSinceCurrKey;

        if (signalStates.find(navigation[i]) != signalStates.end() &&
             (!signalStates.at(navigation[i]) ||
               vehiclesOnLane.find(navigation[i]) != vehiclesOnLane.end() &&
               vehiclesOnLane.at(navigation[i]).begin()->second->velocity < 2))
        {
            // If red light, or traffic on previous state remains in junction, or my lane is jammed
            // don't enter junction
            distanceSinceCurrKey = 0;
            rtn = shared_from_this();
        }
        else
        {
            if (vehiclesOnLane.find(navigation[i]) != vehiclesOnLane.end())
            {
                auto& orderedOnLane = vehiclesOnLane.at(navigation[i]);
                if (orderedOnLane.begin() != orderedOnLane.end())
                {
                    distanceSinceCurrKey = orderedOnLane.begin()->second->s;
                    rtn = orderedOnLane.begin()->second;
                }
            }
            onOverlapZone = GetLeaderInOverlapZone(navigation[i], 0, map, vehiclesOnLane, overlapZones, onOverlapZoneDistance, lookforward);
            if (onOverlapZone != nullptr &&
                (rtn == nullptr || onOverlapZoneDistance < distanceSinceCurrKey))
            {
                rtn = onOverlapZone;
                distanceSinceCurrKey = onOverlapZoneDistance;
            }
        }

        if (rtn != nullptr)
        {
            outDistance += distanceSinceCurrKey;
            assert(outDistance > 0);
            return outDistance < lookforward ? rtn : nullptr;
        }
        outDistance += map.get_lanekey_length(navigation[i]);
    }
    outDistance = lookforward;
    return rtn;
}

double Vehicle::vFromGibbs(double dt, std::shared_ptr<const Vehicle> leader, double distance) const
{
    // Gipps model
    const double tau = 1.5; // reaction time
    const double a = 3; // max acc
    const double b = -8;  // max dcc
    const double s0 = 4;  // static gap
    const double li = DimensionLWH.x();

    double vOut = velocity + 2.5 * a * dt * (1 - velocity / MaxV) *
        std::sqrt(0.025 + velocity / MaxV);

    if (leader != nullptr)
    {
        double underSqr = std::pow(b * dt, 2) - b *
            (-velocity * tau - std::pow(leader->velocity, 2) / b - 2 * li - s0 + 2 * distance);
        if (underSqr < 0)
        {
            // gonna collide, hard stop
            vOut = 0.0;
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

void Vehicle::updateNavigation(const odr::OpenDriveMap& odrMap, const odr::RoutingGraph& routingGraph,
    const std::unordered_map<odr::LaneKey, int>& nVehiclesOnlane)
{
    const auto Source = sourceLane();
    const auto Dest = destLane();

    if (Source.road_id == Dest.road_id && Source.lanesection_s0 == Dest.lanesection_s0
        && Source.lane_id * Dest.lane_id > 0)
    {
        if (sourceS() < destS())
        {
            if (std::equal_to<odr::LaneKey>{}(Source, Dest))
            {
                navigation = { Source };
            }
            else
            {
                navigation = { Source, Dest };
            }
        }
        else
        {
            navigation.clear();
            
            for (auto second : routingGraph.get_lane_successors(Source))
            {
                navigation = routingGraph.shortest_path(second, Dest, nVehiclesOnlane);
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
        navigation = routingGraph.shortest_path(Source, Dest, nVehiclesOnlane);
    }

    if (navigation.empty())
    {
        spdlog::info("No route find form {} @{} to {} @{}", Source.to_string(), sourceS(), 
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
    if (s < laneChangeDueS && lcFrom.has_value())
    {
        double remainingS = laneChangeDueS - s;
        laneChangeRate = tOffset / remainingS;
        tOffset -= laneChangeRate * std::min(dt * velocity, remainingS);
    }
    else
    {
        assert(std::abs(tOffset) < LCCompleteThreshold);
        tOffset = 0;
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
        spdlog::info("Lane {} s={} tOffset={} t={} | G= {} @{} Nav:{} | Stuck:{}", currKey.to_string(), s, tOffset, tCenter + tOffset,
            destLane().to_string(), destS(), navigation.size(), stepInJunction);
    }

    position = newPos;

    double gradFromLane = section.id_to_lane.at(currKey.lane_id).outer_border.get_grad(sOnRefLine + currKey.lanesection_s0);
    double angleFromLane = std::atan2(gradFromLane, 1);
    auto angleFromlaneChange = std::atan2(-laneChangeRate * (reversedTraverse ? -1 : 1), 1);

    auto gradFromRefLine = road.ref_line.get_grad_xy(sOnRefLine + currKey.lanesection_s0);
    auto angleFromRefLine = std::atan2(gradFromRefLine[1], gradFromRefLine[0]);

    heading = angleFromRefLine + angleFromLane + angleFromlaneChange;
    if (reversedTraverse) heading += M_PI;

    auto grad = road.ref_line.elevation_profile.get_grad(sOnRefLine + currKey.lanesection_s0);
    if (reversedTraverse) grad = -grad;

    QMatrix4x4 transformMat;
    transformMat.setToIdentity();
    transformMat.translate(position[0], position[1], position[2]);
    transformMat.rotate(QQuaternion::fromDirection(QVector3D(std::cos(heading), std::sin(heading), grad), QVector3D(0, 0, 1)));
    graphics->SetTransform(transformMat);
    RoadRunner::SpatialIndexerDynamic::Instance()->Index(std::stoi(ID), transformMat, DimensionLWH);
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
    for (int i = 0; i < navigation.size() && i < 3; ++i)
    {
        ss << " [nav " << i << " ]" << navigation[i].to_string() << '\n';
    }
    if (stepInJunction > 0)
    {
        ss << stepInJunction << " steps in current junction\n";
    }
    return ss.str();
}