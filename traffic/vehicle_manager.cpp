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
    routingGraph = RoadRunner::ChangeTracker::Instance()->odrMap.get_routing_graph();
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

void VehicleManager::Spawn()
{
    auto& latestMap = RoadRunner::ChangeTracker::Instance()->odrMap;
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
            auto vehicle = std::make_shared<Vehicle>(startKey, startS, endKey, endS);
            allVehicles.emplace(vehicle->ID, vehicle);
        }
    }
    else
    {
        // Randonly spawn if no route found
        /*
        std::vector<odr::LaneKey> allLanes;
        std::vector<double> allWeights;
        const double MinLengthRequired = 10;

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
        int num = std::ceil(totalLength / 100);
        for (int i = 0; i != num; ++i)
        {
            auto selectedIndex = RandomSelect(allWeights);
            allVehicles.emplace_back(allLanes[selectedIndex], rand01() * allWeights[selectedIndex]);
        }
        */
    }
}

void VehicleManager::step()
{
    vehiclesOnLane.clear();
    for (const auto& id_v : allVehicles)
    {
        for (const auto& laneKey : id_v.second->OccupyingLanes())
        {
            vehiclesOnLane[laneKey].emplace(id_v.second->CurrS(), id_v.second);
        }
    }

    std::set<std::string> inactives;
    for (auto& id_v: allVehicles)
    {
        auto vehicle = id_v.second;
        bool isActive = vehicle->Step(1.0 / FPS, RoadRunner::ChangeTracker::Instance()->odrMap, 
            routingGraph, vehiclesOnLane);
        if (!isActive)
        {
            inactives.emplace(id_v.first);
        }
    }

    for (auto id : inactives)
    {
        allVehicles.at(id)->Clear();
        allVehicles.erase(id);
    }
}
