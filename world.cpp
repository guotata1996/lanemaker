#include "world.h"

World* World::_instance = nullptr;

World* World::Instance()
{
    if (_instance == nullptr)
    {
        _instance = new World();
    }
    return _instance;
}