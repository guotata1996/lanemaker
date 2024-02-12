#pragma once

#include <QGraphicsView>
#include "OpenDriveMap.h"
#include "GameGraphicsScene.h"

//#include <set>
#include <string>
namespace RoadRunner
{
    class MapDrawer : public QGraphicsView
    {
    public:
        MapDrawer(std::string fpath);

        // Apply undrawn changes, if any
        void Update();

        MyGraphicsScene scene;

    private:
        odr::OpenDriveMap odrMap;

        std::string export_path;
    };
}