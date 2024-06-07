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

void IDGenerator::Reset()
{
    IDGenerator::ForRoad()->reset();
    IDGenerator::ForJunction()->reset();
}

size_t IDGenerator::size() const
{
    return assignTo.size();
}

std::string IDGenerator::GenerateID(void* object)
{
    size_t newID = assigned.size();
    for (size_t i = 0; i != assigned.size(); ++i)
    {
        if (!assigned[i])
        {
            assert(assignTo.find(i) == assignTo.end());
            newID = i;
            break;
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
    auto id = static_cast<size_t>(std::atoi(sid.c_str()));
    auto it = assignTo.find(id);
    if (it == assignTo.end())
    {
        throw;
    }
    changeList[id] = it->second;
}

bool IDGenerator::FreeID(const std::string& sid)
{
    auto id = static_cast<size_t>(std::atoi(sid.c_str()));
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

std::map<size_t, void*> IDGenerator::ConsumeChanges()
{
    auto copy = changeList;
    changeList.clear();
    return copy;
}

void IDGenerator::TakeID(std::string sid, void* object)
{
    auto id = static_cast<size_t>(std::atoi(sid.c_str()));
    if (assigned.size() <= id)
    {
        assigned.resize(id + 1, false);
    }
    assert(!assigned[id]);
    assert(assignTo.find(id) == assignTo.end());
    assigned[id] = true;
    assignTo.emplace(id, object);
}

void* IDGenerator::GetByID(std::string sid)
{
    auto id = static_cast<size_t>(std::atoi(sid.c_str()));
    return assignTo.find(id) == assignTo.end() ? nullptr : assignTo.at(id);
}

void IDGenerator::ClearChangeList()
{
    changeList.clear();
}

void IDGenerator::reset()
{
    assigned.clear();
    assignTo.clear();
    changeList.clear();
}