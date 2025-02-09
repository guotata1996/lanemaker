#include "id_generator.h"
#include <cassert>
#include <array>

std::array<std::unique_ptr<IDGenerator>, static_cast<size_t>(IDType::Count)> idStore;


std::unique_ptr<IDGenerator>& IDGenerator::ForType(IDType t)
{
    if (idStore[static_cast<size_t>(t)] == nullptr)
    {
        idStore[static_cast<size_t>(t)] = std::make_unique<IDGenerator>();
    }
    return idStore[static_cast<size_t>(t)];
}

void IDGenerator::Reset()
{
    for (auto& gen : idStore)
    {
        if (gen != nullptr)
        {
            gen->reset();
        }
    }
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
        assert(false);
    }
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