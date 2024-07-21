#include "stats.h"
#include <spdlog/spdlog.h>

Stats::Stats(std::string aName): name(aName)
{
    allInstances.emplace(name, this);
}

Stats::~Stats()
{
    if (name != "_dumpster")
    {
        spdlog::trace("{} : {} times", name, count);
        allInstances.erase(name);
    }
}

void Stats::Increment(int delta)
{
    count += delta;
}

std::map<std::string, Stats*> Stats::allInstances = {};

Stats* Stats::_dumpster = new Stats("_dumpster");


Stats* Stats::Instance(std::string name)
{
    auto it = allInstances.find(name);
    if (it == allInstances.end())
    {
        return _dumpster;
    }
    return it->second;
}

