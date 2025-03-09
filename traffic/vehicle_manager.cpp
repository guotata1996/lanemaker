#include "vehicle_manager.h"
#include "change_tracker.h"
#include "util.h"
#include "map_view_gl.h"
#include "spatial_indexer.h"

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
    if (index >= probs.size()) index = probs.size() - 1;
    return index;
}

int VehicleManager::FPS = 30;

VehicleManager::VehicleManager(QObject* parent): QObject(parent)
{
    timer = new QTimer(this);
    timer->setInterval(FPS);
    connect(timer, SIGNAL(timeout()), this, SLOT(step()));
}

void VehicleManager::Begin()
{
    routingGraph = LM::ChangeTracker::Instance()->Map().get_routing_graph();
    overlapZones = LM::ChangeTracker::Instance()->Map().get_overlap_zones();
    for (auto self_overlaps : overlapZones)
    {
        spdlog::trace("{} overlaps with:", self_overlaps.first.to_string());
        for (auto overlap_and_len : self_overlaps.second)
        {
            spdlog::trace("  {} {}", overlap_and_len.first.to_string(), overlap_and_len.second);
        }
    }
    Spawn();
    for (auto id_junction : LM::ChangeTracker::Instance()->Map().id_to_junction)
    {
        if (id_junction.second.type == odr::JunctionType::Common)
        {
            allSignals.emplace(id_junction.first, std::make_shared<LM::Signal>(id_junction.second));
        }
    }

    stepCount = 0;
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

    for (auto s : allSignals)
    {
        s.second->Terminate();
    }
    allSignals.clear();
    LM::g_mapViewGL->update();
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
    auto& latestMap = LM::ChangeTracker::Instance()->Map();

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
            if (vehicle->GotoNextGoal(LM::ChangeTracker::Instance()->Map(),
                routingGraph, numVehiclesOnLane))
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
        int nPair = std::ceil(totalLength / 100);
        std::cout << "Spawning vehicles ";

        for (auto i: LM::TQDM(LM::range(nPair)))
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
                continue;
            }
            auto maxV = 10 + rand01() * 10;
            auto vehicle = std::make_shared<Vehicle>(startKey, startS, endKey, endS, maxV);

            if (vehicle->GotoNextGoal(LM::ChangeTracker::Instance()->Map(),
                routingGraph, numVehiclesOnLane))
            {
                vehicle->InitGraphics();
                allVehicles.emplace(vehicle->ID, vehicle);
            }
        }
    }
}

void VehicleManager::step()
{
    for (auto id_signal : allSignals)
    {
        id_signal.second->Update(stepCount, signalStateOfLane);
    }

    vehiclesOnLane.clear();
    numVehiclesOnLane.clear();
    for (const auto& id_v : allVehicles)
    {
        for (const auto& laneKey : id_v.second->OccupyingLanes())
        {
            // TODO: conflicting s
            vehiclesOnLane[laneKey].emplace(id_v.second->S(), id_v.second);
            numVehiclesOnLane[laneKey]++;
        }
    }

    std::set<uint32_t> inactives;
    for (auto& id_v: allVehicles)
    {
        auto vehicle = id_v.second;
        bool isActive = vehicle->PlanStep(1.0 / FPS, LM::ChangeTracker::Instance()->Map(),
            vehiclesOnLane, overlapZones, signalStateOfLane);
        if (!isActive)
        {
            inactives.emplace(id_v.first);
        }
    }

    std::vector<uint32_t> to_erase;
    for (auto& id_v : allVehicles)
    {
        auto id = id_v.first;
        if (inactives.find(id) == inactives.end())
        {
            id_v.second->MakeStep(1.0 / FPS, LM::ChangeTracker::Instance()->Map());
        }
        else
        {
            // try reassign goal
            if (!allVehicles.at(id)->GotoNextGoal(LM::ChangeTracker::Instance()->Map(),
                routingGraph, numVehiclesOnLane))
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

    stepCount++;
    LM::g_mapViewGL->update();
}
