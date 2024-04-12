#pragma once

#include <set>
#include "road.h"

class World
{
public:
    std::set<std::shared_ptr<RoadRunner::Road>> allRoads;
    static World* Instance();
    
private:
    static World* _instance;
};