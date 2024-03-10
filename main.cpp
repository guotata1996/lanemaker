#include <QApplication>
#include <QPushButton>
#include <QGraphicsView>
#include <spdlog/spdlog.h>

#include "MapDrawer.h"
#include "road.h"
#include "junction.h"

#include "Geometries/Line.h"
#include "Geometries/Arc.h"
#include "Geometries/Spiral.h"

//#include "test/randomization_utils.h"
//#include "test/junction_verification.h"
//#include "test/road_verification.h"


int main(int argc, char** argv)
{
    spdlog::set_level(spdlog::level::info);
    
    RoadRunner::MapExporter exporter("C:\\Users\\guota\\Downloads\\split.xodr");

    auto refLine1 = std::make_shared<odr::Line>(0, 0, 0, M_PI_2, 40);
    RoadRunner::RoadProfile config(2, 0, 2, 0);
    std::shared_ptr<RoadRunner::Road> r1 = std::make_shared<RoadRunner::Road>(config, refLine1); // (0, 40)

    auto refLine2 = std::make_shared<odr::Arc>(0, 0, 50, M_PI_2, 60, -1 / 20.0); // about (40, 50)
    auto r2 = std::make_shared<RoadRunner::Road>(config, refLine2);
    int err = RoadRunner::Road::JoinRoads(r1, odr::RoadLink::ContactPoint_End, r2, odr::RoadLink::ContactPoint_Start);
    
    auto refLine3 = std::make_shared<odr::Spiral>(0, 90, 10, M_PI / 8, 50, 1 / 60.0, 1 / 30.0);
    auto r3 = std::make_shared<RoadRunner::Road>(config, refLine3);
    err = RoadRunner::Road::JoinRoads(r1, odr::RoadLink::ContactPoint_End, r3, odr::RoadLink::ContactPoint_Start);
    
    auto refLine4 = std::make_shared<odr::Line>(0, 50, 80, 0, 30);
    auto r4 = std::make_shared<RoadRunner::Road>(config, refLine4);
    err = RoadRunner::Road::JoinRoads(r1, odr::RoadLink::ContactPoint_End, r4, odr::RoadLink::ContactPoint_End);
    //auto second = RoadRunner::Road::SplitRoad(r1, 0.65 * r1->Length());
    //second->ReverseRefLine();
    //exporter.Update();

    auto refLine1a = std::make_shared<odr::Line>(0, -20, -15, M_PI, 40);
    auto r1a = std::make_shared<RoadRunner::Road>(config, refLine1a);
    auto refLine1b = std::make_shared<odr::Line>(0, 0, -30, M_PI_2 * 3, 40);
    auto r1b = std::make_shared<RoadRunner::Road>(config, refLine1b);

    auto j1 = std::make_shared<RoadRunner::Junction>();
    j1->CreateFrom({
        RoadRunner::ConnectionInfo{r1, odr::RoadLink::ContactPoint_Start},
        RoadRunner::ConnectionInfo{r1a, odr::RoadLink::ContactPoint_Start},
        RoadRunner::ConnectionInfo{r1b, odr::RoadLink::ContactPoint_Start}
        });
    exporter.Update("C:\\Users\\guota\\Downloads\\step1.xodr");
    
    r1.reset();

    exporter.Update("C:\\Users\\guota\\Downloads\\step2.xodr");

    //QApplication app(argc, argv);
    //RoadRunner::MapDrawer odr_drawer;
    //odr_drawer.Update();
    //odr_drawer.scale(10, 10);
    //odr_drawer.show();
    // return app.exec();
}