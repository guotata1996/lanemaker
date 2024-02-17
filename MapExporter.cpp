#include "MapExporter.h"

#include "IDGenerator.h"
#include "road.h"
#include "junction.h"

#include <spdlog/spdlog.h>

namespace RoadRunner
{
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
                odrMap.id_to_road.emplace(id, static_cast<Road*>(ptr)->generated);
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

    void MapExporter::Update()
    {
        MapUpdater::Update();
        odrMap.export_file(export_path);
    }
}