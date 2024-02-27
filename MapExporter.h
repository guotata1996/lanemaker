#pragma once

#include "OpenDriveMap.h"

#include <string>
namespace RoadRunner
{
    class MapUpdater
    {
    public:
        void Update();

    protected:
        odr::OpenDriveMap odrMap;
    };

    class MapExporter: public MapUpdater
    {
    public:
        MapExporter(std::string fpath);

        void Update(std::string overridePath="");
    private:
        std::string export_path;
    };
}