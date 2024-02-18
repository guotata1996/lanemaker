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

    RoadRunner::RoadProfile configs(1, 1, 1, 0);
    configs.OverwriteSection(-1, 20, 60, 2, 0);
    configs.OverwriteSection(1, 60, 15, 2, 1);
    configs.OverwriteSection(1, 25, 10, 2, 2);

    // RoadRunnerTest::GenerateAndVerify(profile);

    auto defaultRefLine = std::make_shared<odr::Line>(0, 0, 0, 0, 40);
    RoadRunner::Road road(configs, defaultRefLine);
    road.Generate();
    RoadRunner::MapExporter exporter("C:\\Users\\guota\\Downloads\\test.xodr");
    exporter.Update();
    
    // allJunctions.clear();
    //QApplication app(argc, argv);
    //RoadRunner::MapDrawer odr_drawer;
    //odr_drawer.Update();
    //odr_drawer.scale(10, 10);
    //odr_drawer.show();
    // return app.exec();
}