#include <QApplication>
#include <QPushButton>
#include <QGraphicsView>
#include <spdlog/spdlog.h>

#include "MapDrawer.h"
#include "road.h"
//#include "test/randomization_utils.h"
//#include "test/junction_verification.h"
//#include "test/road_verification.h"


int main(int argc, char** argv)
{
    spdlog::set_level(spdlog::level::info);

    RoadRunner::RoadProfile configs(1, 1, 0, 0);
    //configs.OverwriteSection(-1, 10.0, 30.0, 2, -2);
    configs.OverwriteSection(1, 50.8, 20, 3, 3);

    auto refLine = std::make_shared<odr::Line>(0, 0, 0, M_PI_4, 50);
    RoadRunner::Road r(configs, refLine);

    r.ReverseRefLine();
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