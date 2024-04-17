#include "change_tracker.h"
#include "id_generator.h"
#include "road.h"
#include "junction.h"

#include <spdlog/spdlog.h>

namespace RoadRunner
{
    /*
    void MapUpdater::Update()
    {
        for (auto& change : IDGenerator::ForRoad()->ConsumeChanges())
        {
            std::string id = std::to_string(change.first);
            if (odrMap.id_to_road.find(id) != odrMap.id_to_road.end())
            {
                odrMap.id_to_road.erase(id);
                spdlog::trace("erase road {} from exportMap", id);
            }
            void* ptr = change.second;
            if (ptr != nullptr)
            {
                Road* r = static_cast<Road*>(ptr);
                auto& gen = r->generated;
                odrMap.id_to_road.emplace(id, gen);
            }
        }

        for (auto& change : IDGenerator::ForJunction()->ConsumeChanges())
        {
            std::string id = std::to_string(change.first);
            if (odrMap.id_to_junction.find(id) != odrMap.id_to_junction.end())
            {
                odrMap.id_to_junction.erase(id);
                spdlog::trace("erase junction {} from exportMap", id);
            }
            void* ptr = change.second;
            if (ptr != nullptr)
            {
                odrMap.id_to_junction.emplace(id, static_cast<Junction*>(ptr)->generated);
            }
        }
    }

    MapExporter::MapExporter(std::string fpath) : export_path(fpath) {}

    void MapExporter::Update(std::string overridePath)
    {
        MapUpdater::Update();
        if (overridePath.empty())
        {
            odrMap.export_file(export_path);
        }
        else
        {
            odrMap.export_file(overridePath);
        }
    }
    */

    ChangeTracker* ChangeTracker::instance = nullptr;

    ChangeTracker* ChangeTracker::Instance()
    {
        if (instance == nullptr)
        {
            instance = new ChangeTracker;
        }
        return instance;
    }

    void ChangeTracker::Record()
    {
        auto roadChanges = IDGenerator::ForRoad()->ConsumeChanges();
        auto junctionChanges = IDGenerator::ForJunction()->ConsumeChanges();
        if (roadChanges.empty() && junctionChanges.empty())
        {
            return;
        }

        MapChange recordEntry;

        for (const auto& change : roadChanges)
        {
            std::string id = std::to_string(change.first);
            void* ptr = change.second;

            RoadChange roadChange;
            auto oldIt = odrMap.id_to_road.find(id);
            if (oldIt != odrMap.id_to_road.end())
            {
                roadChange.before.emplace(oldIt->second);
                odrMap.id_to_road.erase(id);
            }

            if (ptr != nullptr)
            {
                Road* newRoad = static_cast<Road*>(ptr);
                roadChange.after.emplace(newRoad->generated);
                odrMap.id_to_road.emplace(newRoad->ID(), newRoad->generated);
            }
            recordEntry.roadChanges.push_back(roadChange);
        }
        
        for (const auto& change : junctionChanges)
        {
            std::string id = std::to_string(change.first);
            void* ptr = change.second;

            JunctionChange junctionChange;
            auto oldIt = odrMap.id_to_junction.find(id);
            if (oldIt != odrMap.id_to_junction.end())
            {
                junctionChange.before.emplace(oldIt->second);
                odrMap.id_to_junction.erase(id);
            }

            if (ptr != nullptr)
            {
                Junction* newJunction = static_cast<Junction*>(ptr);
                junctionChange.after.emplace(newJunction->generated);
                odrMap.id_to_junction.emplace(newJunction->ID(), newJunction->generated);
            }
            recordEntry.junctionChanges.push_back(junctionChange);
        }
        
        // TODO: Fully serialize Road(refLine) and Junction
    }

    void ChangeTracker::Save(std::string path)
    {

    }
}