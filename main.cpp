#include <fstream>

#include <QApplication>
#include <QPushButton>
#include <QGraphicsView>
#include <string>

#include "junction.h"

#include "GameGraphicsScene.h"

#include "verification.h"
#include "test_randomization.h"

#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
#include "CubicBezier.hpp"


int main(int argc, char** argv)
{
    
    spdlog::set_level(spdlog::level::info);

    //odr::Line l1(0, 0, -50, M_PI / 2, 20);
    //odr::Line l2(0, 30, -20, M_PI * 7 / 4, 20);
    //// odr::Line l2(0, 50, -20, M_PI * 5 / 4, 20);
    //auto connectingRefLine = ConnectLines(l1, l2);
    //if (connectingRefLine.has_value())
    //{
    //    odr::OpenDriveMap test_map;
    //    RoadRunner::Road connecting("conn");
    //    connecting.SetLength(connectingRefLine.value().length * 100);
    //    connecting.AddRightSection({ RoadRunner::RoadProfile{1, 1}, 0 });
    //    odr::Road connectingRoad = (odr::Road)connecting;
    //    connectingRoad.ref_line.s0_to_geometry[0] = connectingRefLine.value().clone();
    //    test_map.id_to_road.insert({ "1", connectingRoad });
    //    test_map.export_file("C:\\Users\\guota\\Downloads\\junction.xodr");
    //}

    RoadRunner::Road sourceCfg("source");
    sourceCfg.SetLength(50 * 100);
    sourceCfg.AddRightSection({ RoadRunner::RoadProfile{0, 2} , 0 * 100 });
    odr::Road source = (odr::Road)sourceCfg;

    RoadRunner::Road sinkCfg("lsink");
    sinkCfg.SetLength(50 * 100);
    sinkCfg.AddRightSection({ RoadRunner::RoadProfile{-1, 1}, 0 * 100 });
    sinkCfg.AddLeftSection({ RoadRunner::RoadProfile{1, 1}, 100 * 100 });

    odr::Road leftSink = (odr::Road)sinkCfg;
    odr::Road rightSink = (odr::Road)sinkCfg;
    rightSink.id = "rsink";

    source.ref_line.s0_to_geometry[0] = std::make_unique<odr::Line>(0, 0, -60, M_PI / 2, source.length);
    leftSink.ref_line.s0_to_geometry[0] = std::make_unique<odr::Line>(0, -60, 0, 0, leftSink.length);
    rightSink.ref_line.s0_to_geometry[0] = std::make_unique<odr::Line>(0, 60, 0, M_PI, rightSink.length);

    std::vector< odr::Road> connectings;
    auto junction1 = GenerateConnections({ 
        ConnectionInfo{sourceCfg, &source, source.length},
        ConnectionInfo{sinkCfg, &rightSink, leftSink.length},
        ConnectionInfo{sinkCfg, &leftSink, rightSink.length} },
        connectings);

    odr::OpenDriveMap test_map;
    test_map.id_to_road.insert({ source.id, source });
    test_map.id_to_road.insert({ rightSink.id, rightSink });
    test_map.id_to_road.insert({ leftSink.id, leftSink });
    for (auto connecting : connectings)
    {
        test_map.id_to_road.insert({ connecting.id, connecting });
    }

    test_map.id_to_junction.insert({ junction1.id , junction1 });
    
    test_map.export_file("C:\\Users\\guota\\Downloads\\junction.xodr");

    //QApplication app(argc, argv);
    //MyGraphicsScene scene;
    //scene.DrawXodr(test_map);
    //QGraphicsView view(&scene);
    //view.scale(15, 15);
    //view.show();

    // return app.exec();
}