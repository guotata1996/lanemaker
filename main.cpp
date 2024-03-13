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

    auto refLine1 = std::make_shared<odr::Line>(0, -30, 0, 0, 60);
    RoadRunner::RoadProfile config(2, 1, 2, -1);
    std::shared_ptr<RoadRunner::Road> r1 = std::make_shared<RoadRunner::Road>(config, refLine1); // (0, 40)

    const double R = 30;
    auto refLine2 = std::make_shared<odr::Arc>(0, 50, 20, M_PI_4, 0.75 * M_PI * 2 * R, -1 / R); // about (40, 50)
    auto r2 = std::make_shared<RoadRunner::Road>(config, refLine2);
    //int err = RoadRunner::Road::JoinRoads(r1, odr::RoadLink::ContactPoint_End, r2, odr::RoadLink::ContactPoint_Start);
    
    auto refLine3 = std::make_shared<odr::Arc>(0, -50, 20, M_PI_4 * 3, 0.75 * M_PI * 2 * R, 1 / R);
    auto r3 = std::make_shared<RoadRunner::Road>(config, refLine3);
    //err = RoadRunner::Road::JoinRoads(r1, odr::RoadLink::ContactPoint_End, r3, odr::RoadLink::ContactPoint_Start);
        
    std::make_shared<RoadRunner::Junction>()->CreateFrom({
        RoadRunner::ConnectionInfo{r1, odr::RoadLink::ContactPoint_End},
        RoadRunner::ConnectionInfo{r2, odr::RoadLink::ContactPoint_Start},
        RoadRunner::ConnectionInfo{r2, odr::RoadLink::ContactPoint_End}
        });

    std::make_shared<RoadRunner::Junction>()->CreateFrom({
        RoadRunner::ConnectionInfo{r1, odr::RoadLink::ContactPoint_Start},
        RoadRunner::ConnectionInfo{r3, odr::RoadLink::ContactPoint_Start},
        RoadRunner::ConnectionInfo{r3, odr::RoadLink::ContactPoint_End}
        });

    exporter.Update("C:\\Users\\guota\\Downloads\\step1.xodr");
    
    // r1->ReverseRefLine();
    auto part3 = RoadRunner::Road::SplitRoad(r1, 40);
    auto part2 = RoadRunner::Road::SplitRoad(r1, 20);
    part2.reset();
    exporter.Update("C:\\Users\\guota\\Downloads\\step2.xodr");

    RoadRunner::Road::JoinRoads(r1, odr::RoadLink::ContactPoint_End, part3, odr::RoadLink::ContactPoint_Start);
    exporter.Update("C:\\Users\\guota\\Downloads\\step3.xodr");
    r1->ReverseRefLine();
    exporter.Update("C:\\Users\\guota\\Downloads\\step4.xodr");
    r1.reset();
    r2.reset();
    exporter.Update("C:\\Users\\guota\\Downloads\\step5.xodr");

    //QApplication app(argc, argv);
    //RoadRunner::MapDrawer odr_drawer;
    //odr_drawer.Update();
    //odr_drawer.scale(10, 10);
    //odr_drawer.show();
    // return app.exec();
}