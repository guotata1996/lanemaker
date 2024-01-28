#include <fstream>

#include <QApplication>
#include <QPushButton>
#include <QGraphicsView>
#include <string>

#include "road.h"

#include "GameGraphicsScene.h"

#include "verification.h"


int main(int argc, char** argv)
{
    
    // odr::OpenDriveMap odr_map("C:\\Users\\guota\\Downloads\\lane_offset.xodr");
    // odr_map.export_file("C:\\Users\\guota\\Downloads\\export.xodr");
    spdlog::set_level(spdlog::level::debug);
    odr::OpenDriveMap test_map;

    RoadRunner::Road road("");
    road.SetLength(90 * 100);

    RoadRunner::LaneSection ls1{ RoadRunner::RoadProfile{1, 1}, 90 * 100 };
    RoadRunner::LaneSection ls2{ RoadRunner::RoadProfile{0, 1}, 30 * 100 };
    road.AddLeftSection(ls1);
    road.AddLeftSection(ls2);

    odr::Road gen = (odr::Road)road;

    VerifyLaneWidthinBound(gen);
    VerifySingleRoadLinkage(gen);
    VerifySingleRoadIntegrity(road, gen);

    //QApplication app(argc, argv);
    //MyGraphicsScene scene;
    //scene.DrawXodr(test_map);
    //QGraphicsView view(&scene);
    //view.scale(15, 15);
    //view.show();

    // return app.exec();
}