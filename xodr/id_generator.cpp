#include "id_generator.h"
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

size_t IDGenerator::size() const
{
    return assignTo.size();
}

std::string IDGenerator::GenerateID(void* object)
{
    IDType newID = assigned.size();
    for (IDType i = 0; i != assigned.size(); ++i)
    {
        if (!assigned[i])
        {
            assert(assignTo.find(i) == assignTo.end());
            newID = i;
        }
    }

    if (newID == assigned.size())
    {
        assigned.push_back(true);
    }
    else
    {
        assigned[newID] = true;
    }

    assignTo.emplace(newID, object);
    spdlog::trace("Geneated {} ID: {}", type, newID);
    changeList[newID] = object;
    return std::to_string(newID);
}

void IDGenerator::NotifyChange(const std::string& sid)
{
    auto id = static_cast<IDType>(std::atoi(sid.c_str()));
    auto it = assignTo.find(id);
    if (it == assignTo.end())
    {
        throw;
    }
    changeList[id] = it->second;
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
    changeList[id] = nullptr;
    return true;
}

std::map<IDGenerator::IDType, void*> IDGenerator::ConsumeChanges()
{
    auto copy = changeList;
    changeList.clear();
    return copy;
}