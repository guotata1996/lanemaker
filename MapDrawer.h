#pragma once

#include "OpenDriveMap.h"

//#include <set>
#include <string>
namespace RoadRunner
{
    class MapDrawer
    {
    public:
        MapDrawer(std::string fpath);

        // Apply undrawn changes, if any
        void Update();
    private:
        odr::OpenDriveMap odrMap;

        std::string export_path;

        // std::set<std::string> currentRoads, currentJunctions;
    };
}