#include <QApplication>
#include <QPushButton>
#include <QGraphicsView>
#include <spdlog/spdlog.h>

#include "MapDrawer.h"
#include "road.h"
#include "test/randomization_utils.h"
#include "test/junction_verification.h"
#include "test/road_verification.h"


int main(int argc, char** argv)
{
    spdlog::set_level(spdlog::level::info);

    //RoadRunner::RoadProfile configs(1, 1, 1, 0);
    //configs.OverwriteSection(-1, 20, 60, 2, 0);
    //configs.OverwriteSection(1, 60, 15, 2, 1);
    //configs.OverwriteSection(1, 25, 10, 2, 2);

    //const uint32_t Length_M = 100;
    //auto road = GenerateConfig(1, Length_M);
    //RoadRunnerTest::GenerateAndVerify(configs, Length_M);

    srand(1);

    const int NumRoads = RandomIntBetween(3, 6);
    const double SeparationAngle = M_PI * 2 / NumRoads;
    const uint32_t RoadLength = 30 * 100;
    const double RoadLengthD = RoadRunner::to_odr_unit(RoadLength);
    const odr::Vec2D nearEnd{ 25, 0 };
    const odr::Vec2D farEnd{ 55, 0 };

    std::vector< RoadRunner::Road> generatedRoads;
    std::vector< bool > incomings;

    int totalOuts = 0;
    std::vector < int> numIncominglanes;
    for (int i = 0; i != NumRoads; ++i)
    {
        int8_t rightLanes = RandomIntBetween(1, 4);
        int8_t leftLanes = RandomIntBetween(1, 4);
        int8_t rightOffset = RandomIntBetween(-1, 0);
        int8_t leftOffset = RandomIntBetween(0, 1);
        bool incoming = RandomIntBetween(0, 1) == 0;  //refLane pointing junction
        totalOuts += incoming ? leftLanes : rightLanes;
        numIncominglanes.push_back(incoming ? rightLanes : leftLanes);

        RoadRunner::RoadProfile cfg(leftLanes, leftOffset, rightLanes, rightOffset);

        odr::Vec2D refLineOrigin = incoming ? farEnd : nearEnd;
        refLineOrigin = odr::rotateCCW(refLineOrigin, SeparationAngle * i);
        double hdg = SeparationAngle * i;
        if (incoming) hdg += M_PI;
        auto refLine = std::make_shared<odr::Line>(0, refLineOrigin[0], refLineOrigin[1], hdg, RoadLengthD);

        RoadRunner::Road road(cfg, refLine);
        road.Generate();
        generatedRoads.push_back(std::move(road));
        incomings.push_back(incoming);
    }

    std::vector< RoadRunner::ConnectionInfo> connectionInfo;
    for (int i = 0; i != NumRoads; ++i)
    {
        connectionInfo.push_back(RoadRunner::ConnectionInfo{
            &generatedRoads[i], incomings[i] ? RoadLengthD : 0 });
    }

    RoadRunner::Junction j1(connectionInfo);
    spdlog::info("Junction error = {}", j1.generationError);

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