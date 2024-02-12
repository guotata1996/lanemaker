#include <fstream>

#include <QApplication>
#include <QPushButton>
#include <QGraphicsView>
#include <string>

#include "junction.h"

// #include "verification.h"
// #include "test_randomization.h"

#include <spdlog/spdlog.h>

#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
#include "CubicBezier.hpp"

#include "MapDrawer.h"

void FourRoads(std::vector<RoadRunner::Road>& allRoads)
{
    RoadRunner::RoadProfile vConfig(40 * 100);
    vConfig.AddRightSection({ RoadRunner::SectionProfile{-1, 3} , 0 * 100 });
    vConfig.AddLeftSection({ RoadRunner::SectionProfile{1, 2} , 40 * 100 });
    RoadRunner::RoadProfile hConfig(40 * 100);
    hConfig.AddRightSection({ RoadRunner::SectionProfile{-1, 1}, 0 * 100 });
    hConfig.AddLeftSection({ RoadRunner::SectionProfile{1, 2}, 40 * 100 });

    RoadRunner::Road upper(vConfig);
    RoadRunner::Road bottom(vConfig);

    RoadRunner::Road left(hConfig);
    RoadRunner::Road right(hConfig);

    bottom.Generate(odr::Line(0, 0, -60, M_PI / 2, bottom.Length()));
    upper.Generate(odr::Line(0, 0, 20, M_PI / 2, upper.Length()));
    left.Generate(odr::Line(0, -60, 0, 0, left.Length()));
    right.Generate(odr::Line(0, 20, 0, 0, right.Length()));

    allRoads.push_back(std::move(upper));
    allRoads.push_back(std::move(bottom));
    allRoads.push_back(std::move(left));
    allRoads.push_back(std::move(right));
}

int main(int argc, char** argv)
{
    spdlog::set_level(spdlog::level::info);

    std::vector<RoadRunner::Road> allRoads;
    std::vector<RoadRunner::Junction> allJunctions;
    FourRoads(allRoads);

    RoadRunner::Junction j1({
        RoadRunner::ConnectionInfo{&allRoads[0], 0},
        RoadRunner::ConnectionInfo{&allRoads[1], allRoads[1].Length()},
        RoadRunner::ConnectionInfo{&allRoads[2], allRoads[2].Length()},
        RoadRunner::ConnectionInfo{&allRoads[3], 0}});
    allJunctions.push_back(std::move(j1));
    
    // allJunctions.clear();
    QApplication app(argc, argv);
    RoadRunner::MapDrawer odr_writer("C:\\Users\\guota\\Downloads\\junction.xodr");
    odr_writer.scale(10, 10);
    odr_writer.show();

    return app.exec();
}