#include <QApplication>
#include <QPushButton>
#include <QGraphicsView>
#include <spdlog/spdlog.h>

#include "road.h"
#include "junction.h"

#include "Geometries/Line.h"
//#include "Geometries/Arc.h"
//#include "Geometries/Spiral.h"

//#include "test/randomization_utils.h"
//#include "test/junction_verification.h"
//#include "test/road_verification.h"
#include "world.h"
#include "mainwindow.h"

int main(int argc, char** argv)
{
    spdlog::set_level(spdlog::level::info);

    QApplication app(argc, argv);
    app.setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);

    MainWindow window;
    window.show();

    auto refLine1 = std::make_shared<odr::Line>(0, -30, 0, 0, 60);
    RoadRunner::RoadProfile config(2, 1, 2, -1);
    config.OverwriteSection(-1, 20.0, 30.0, 3, 0);
    std::shared_ptr<RoadRunner::Road> r1 = std::make_shared<RoadRunner::Road>(config, refLine1);
    r1->GenerateAllSectionGraphics();
    World::Instance()->allRoads.insert(r1);

    return app.exec();
}