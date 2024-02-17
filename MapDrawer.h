#pragma once

#include <QGraphicsView>
#include "OpenDriveMap.h"
#include "GameGraphicsScene.h"
#include "MapExporter.h"

#include <string>
namespace RoadRunner
{
    class MapDrawer : public MapUpdater, public QGraphicsView
    {
    public:
        MapDrawer();
        // Apply undrawn changes, if any
        void Update();

    private:
        MyGraphicsScene scene;
    };
}