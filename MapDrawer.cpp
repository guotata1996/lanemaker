#include "MapDrawer.h"


#include <spdlog/spdlog.h>

namespace RoadRunner
{
    MapDrawer::MapDrawer()
    {
        setScene(&scene);
    }

    void MapDrawer::Update()
    {
        MapUpdater::Update();
        scene.DrawXodr(odrMap); // TODO: draw incremantly
    }
}