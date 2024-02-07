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

    RoadRunner::Road vConfig("bottom");
    vConfig.SetLength(40 * 100);
    vConfig.AddRightSection({ RoadRunner::RoadProfile{-1, 3} , 0 * 100 });
    vConfig.AddLeftSection({ RoadRunner::RoadProfile{1, 2} , 40 * 100 });
    RoadRunner::Road hConfig("left");
    hConfig.SetLength(40 * 100);
    hConfig.AddRightSection({ RoadRunner::RoadProfile{-1, 1}, 0 * 100 });
    hConfig.AddLeftSection({ RoadRunner::RoadProfile{1, 2}, 40 * 100 });

    odr::Road bottom = (odr::Road)vConfig;
    odr::Road upper = (odr::Road)vConfig;
    upper.id = "upper";

    odr::Road left = (odr::Road)hConfig;
    odr::Road right = (odr::Road)hConfig;
    right.id = "right";

    bottom.ref_line.s0_to_geometry[0] = std::make_unique<odr::Line>(0, 0, -60, M_PI / 2, bottom.length);
    upper.ref_line.s0_to_geometry[0] = std::make_unique<odr::Line>(0, 0, 20, M_PI / 2, upper.length);
    left.ref_line.s0_to_geometry[0] = std::make_unique<odr::Line>(0, -60, 0, 0, left.length);
    right.ref_line.s0_to_geometry[0] = std::make_unique<odr::Line>(0, 20, 0, 0, right.length);

    std::vector< odr::Road> connectings;
    auto junction1 = GenerateConnections({ 
        ConnectionInfo{vConfig, &bottom, bottom.length},
        ConnectionInfo{vConfig, &upper, 0},
        ConnectionInfo{hConfig, &left, left.length},
        ConnectionInfo{hConfig, &right, 0} },
        connectings);

    odr::OpenDriveMap test_map;
    test_map.id_to_road.insert({ bottom.id, bottom });
    test_map.id_to_road.insert({ upper.id, upper });
    test_map.id_to_road.insert({ right.id, right });
    test_map.id_to_road.insert({ left.id, left });
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