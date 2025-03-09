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

uint32_t IDGenerator::GenerateID(void* object)
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
    return newID;
}

void IDGenerator::NotifyChange(uint32_t id)
{
    auto it = assignTo.find(id);
    if (it == assignTo.end())
    {
        throw;
    }
    changeList[id] = it->second;
}

bool IDGenerator::FreeID(uint32_t id)
{
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

void IDGenerator::TakeID(uint32_t id, void* object)
{
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