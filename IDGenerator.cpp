#include "IDGenerator.h"
#include <cassert>
#include <spdlog/spdlog.h>

IDGenerator* IDGenerator::_junction = nullptr;
IDGenerator* IDGenerator::_road = nullptr;

IDGenerator::IDGenerator(std::string aType): type(aType)
{
    
}

IDGenerator* IDGenerator::ForJunction()
{
    if (_junction == nullptr)
    {
        _junction = new IDGenerator("Junc");
    }
    return _junction;
}

IDGenerator* IDGenerator::ForRoad()
{
    if (_road == nullptr)
    {
        _road = new IDGenerator("Road");
    }
    return _road;
}

std::string IDGenerator::GenerateID(void* object)
{
    for (uint32_t i = 0; i != assigned.size(); ++i)
    {
        if (!assigned[i])
        {
            assert(assignTo.find(i) == assignTo.end());
            assigned[i] = true;
            assignTo.emplace(i, object);
            spdlog::trace("Geneated {} ID: {}", type, i);
            return std::to_string(i);
        }
    }

    uint32_t newID = assigned.size();
    assigned.push_back(true);
    assigned[newID] = true;
    assignTo.emplace(newID, object);
    spdlog::trace("Geneated {} ID: {}", type, newID);
    return std::to_string(newID);
}

void* IDGenerator::GetByID(const std::string& sid)
{
    auto id = static_cast<uint32_t>(std::atoi(sid.c_str()));
    return assignTo.at(id);
}

bool IDGenerator::FreeID(const std::string& sid)
{
    auto id = static_cast<uint32_t>(std::atoi(sid.c_str()));
    if (assigned.size() <= id || !assignTo.at(id))
    {
        // ID does not exist
        spdlog::warn("Can't free non-exist {} ID: {}", type, sid);
        return false;
    }
    spdlog::trace("Free {} ID: {}", type, sid);
    assigned[id] = false;
    assignTo.erase(id);
    return true;
}