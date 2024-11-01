#include "vehicle_manager.h"
#include "change_tracker.h"

#include "spdlog/spdlog.h"

double rand01()
{
    return static_cast<double>(rand()) / RAND_MAX;
}

size_t RandomSelect(const std::vector<double>& probs)
{
    std::vector<double> sumWeights = { 0 };
    for (double p : probs)
    {
        sumWeights.push_back(sumWeights.back() + p);
    }
    double target = rand01() * sumWeights.back();
    
    auto it = std::upper_bound(sumWeights.begin(), sumWeights.end(), target);
    size_t index = std::distance(sumWeights.begin(), it);
    if (index != 0) index--;
    return index;
}

VehicleManager::VehicleManager(QObject* parent): QObject(parent),
    idGen(IDGenerator::ForVehicle())
{
    timer = new QTimer(this);
    timer->setInterval(FPS);
    connect(timer, SIGNAL(timeout()), this, SLOT(step()));
}

void VehicleManager::Begin()
{
    routingGraph = RoadRunner::ChangeTracker::Instance()->Map().get_routing_graph();
    overlapZones = RoadRunner::ChangeTracker::Instance()->Map().get_overlap_zones();
    for (auto self_overlaps : overlapZones)
    {
        spdlog::trace("{} overlaps with:", self_overlaps.first.to_string());
        for (auto overlap_and_len : self_overlaps.second)
        {
            spdlog::trace("  {} {}", overlap_and_len.first.to_string(), overlap_and_len.second);
        }
    }
    Spawn();
    timer->start();
}

void VehicleManager::End()
{
    timer->stop();
    for (auto v : allVehicles)
    {
        v.second->Clear();
    }
    allVehicles.clear();
}

void VehicleManager::TogglePause()
{
    if (timer->isActive())
    {
        timer->stop();
    }
    else
    {
        timer->start();
    }
}

void VehicleManager::Spawn()
{
    auto& latestMap = RoadRunner::ChangeTracker::Instance()->Map();
    auto& latestRoutingGraph = latestMap.get_routing_graph();

    auto setRoutes = latestMap.get_routes();
    if (!setRoutes.empty())
    {
        for (const auto& start_end : setRoutes)
        {
            auto startKey = std::get<0>(start_end);
            auto startS = std::get<1>(start_end);
            auto endKey = std::get<2>(start_end);
            auto endS = std::get<3>(start_end);
            auto vehicle = std::make_shared<Vehicle>(startKey, startS, endKey, endS, 
                allVehicles.size() % 2 == 1 ? 12 : 20);
            if (vehicle->GotoNextGoal(RoadRunner::ChangeTracker::Instance()->Map(),
                routingGraph))
            {
                vehicle->InitGraphics();
                allVehicles.emplace(vehicle->ID, vehicle);
            }
            else
            {
                spdlog::info("Routing fails");
            }
        }
    }
    else
    {
        int seed = rand();
        srand(seed);
        spdlog::info("Spawn seed = {}", seed);
        // Randonly spawn if no route found
        std::vector<odr::LaneKey> allLanes;
        std::vector<double> allWeights;
        const double MinLengthRequired = 10; // TODO: this should depend on number of lanes to limit lane change rate

        for (auto id_road : latestMap.id_to_road)
        {
            if (id_road.second.junction != "-1") continue;
            for (auto id_section : id_road.second.s_to_lanesection)
            {
                double length = id_road.second.get_lanesection_length(id_section.first);
                if (length < MinLengthRequired) continue;

                for (auto id_lane : id_section.second.id_to_lane)
                {
                    odr::Lane lane = id_lane.second;
                    if (lane.type != "driving") continue;
                    allLanes.push_back(lane.key);
                    allWeights.push_back(length - MinLengthRequired);
                }
            }
        }

        if (allLanes.empty())
        {
            spdlog::warn("No roads to spawn on! Try creating longer roads.");
            return;
        }

        double totalLength = std::accumulate(allWeights.begin(), allWeights.end(), 0);
        int nPair = std::ceil(totalLength / 50);
        bool showProgress = nPair > 100;
        for (int i = 0; i != nPair; ++i)
        {
            auto startIndex = RandomSelect(allWeights);
            auto endIndex = RandomSelect(allWeights);

            auto startKey = allLanes[startIndex];
            auto endKey = allLanes[endIndex];
            // At least MinLengthRequired / 2 from both ends
            auto startS = rand01() * allWeights[startIndex] + MinLengthRequired / 2;
            auto endS = rand01() * allWeights[endIndex] + MinLengthRequired / 2;

            if (startKey.road_id == endKey.road_id && startKey.lanesection_s0 == endKey.lanesection_s0
                && startKey.lane_id != endKey.lane_id && startKey.lane_id * endKey.lane_id > 0
                && std::abs(startS - endS) < MinLengthRequired / 2)
            {
                // Reject abrupt lane change req.
                i--;
                continue;
            }
            auto maxV = 10 + rand01() * 10;
            auto vehicle = std::make_shared<Vehicle>(startKey, startS, endKey, endS, maxV);

            if (vehicle->GotoNextGoal(RoadRunner::ChangeTracker::Instance()->Map(),
                routingGraph))
            {
                vehicle->InitGraphics();
                allVehicles.emplace(vehicle->ID, vehicle);
            }
        }
    }
}

void VehicleManager::step()
{
    vehiclesOnLane.clear();
    for (const auto& id_v : allVehicles)
    {
        for (const auto& laneKey : id_v.second->OccupyingLanes())
        {
            // TODO: conflicting s
            vehiclesOnLane[laneKey].emplace(id_v.second->CurrS(), id_v.second);
        }
    }

    std::set<std::string> inactives;
    for (auto& id_v: allVehicles)
    {
        auto vehicle = id_v.second;
        bool isActive = vehicle->PlanStep(1.0 / FPS, RoadRunner::ChangeTracker::Instance()->Map(),
            vehiclesOnLane, overlapZones);
        if (!isActive)
        {
            inactives.emplace(id_v.first);
        }
    }

    std::vector<std::string> to_erase;
    for (auto& id_v : allVehicles)
    {
        auto id = id_v.first;
        if (inactives.find(id) == inactives.end())
        {
            id_v.second->MakeStep(1.0 / FPS, RoadRunner::ChangeTracker::Instance()->Map());
        }
        else
        {
            // try reassign goal
            if (!allVehicles.at(id)->GotoNextGoal(RoadRunner::ChangeTracker::Instance()->Map(),
                routingGraph))
            {
                allVehicles.at(id)->Clear();
                to_erase.push_back(id);
            }
        }
    }

    for (auto id : to_erase)
    {
        allVehicles.erase(id);
    }
}
