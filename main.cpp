#include <QApplication>
#include <QPushButton>
#include <QGraphicsView>
#include <spdlog/spdlog.h>

#include "MapDrawer.h"
#include "road.h"

#include "Geometries/Line.h"
#include "Geometries/Spiral.h"

//#include "test/randomization_utils.h"
//#include "test/junction_verification.h"
//#include "test/road_verification.h"


int main(int argc, char** argv)
{
    spdlog::set_level(spdlog::level::info);

    RoadRunner::RoadProfile config1(1, 1, 2, 0);
    //configs.OverwriteSection(-1, 10.0, 30.0, 2, -2);
    //configs.OverwriteSection(1, 50.8, 20, 3, 3);
    auto refLine1 = std::make_shared<odr::Line>(0, 20, 0, M_PI_2 * 3, 60);
    RoadRunner::Road r1(config1, refLine1);
    
    {
        RoadRunner::RoadProfile config2(1, 0, 2, 0);
        auto refLine2 = std::make_shared<odr::Line>(0, 40, 40, M_PI_4, 40);
        RoadRunner::Road r2(config2, refLine2);
        RoadRunner::Road::JoinRoads(&r1, odr::RoadLink::ContactPoint_Start, 
            &r2, odr::RoadLink::ContactPoint_Start);
        // r2 goes out of scope
    }
    r1.ReverseRefLine();
    RoadRunner::MapExporter exporter("C:\\Users\\guota\\Downloads\\reverse.xodr");
    exporter.Update();
    
    // allJunctions.clear();
    //QApplication app(argc, argv);
    //RoadRunner::MapDrawer odr_drawer;
    //odr_drawer.Update();
    //odr_drawer.scale(10, 10);
    //odr_drawer.show();
    // return app.exec();
}