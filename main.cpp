#include <fstream>


#include <QApplication>
#include <QPushButton>
#include <QGraphicsView>

#include <string>

#include "road.h"

#include "GameGraphicsScene.h"


int main(int argc, char** argv)
{
    
    // odr::OpenDriveMap odr_map("C:\\Users\\guota\\Downloads\\lane_offset.xodr");
    // odr_map.export_file("C:\\Users\\guota\\Downloads\\export.xodr");

    odr::OpenDriveMap test_map;

    RoadRunnder::Road road("1");
    road.SetLength(150 * 100);
    road.AddRightSection({ RoadRunnder::RoadProfile{-1, 1}, 0 * 100 });
    road.AddRightSection({ RoadRunnder::RoadProfile{1, 2}, 60 * 100 });
    road.AddRightSection({ RoadRunnder::RoadProfile{-1, 1}, 90 * 100 });

    road.AddLeftSection({ RoadRunnder::RoadProfile{-1, 2}, 0 * 100 });
    road.AddLeftSection({ RoadRunnder::RoadProfile{1, 1}, 30 * 100 });
    road.AddLeftSection({ RoadRunnder::RoadProfile{-1, 2}, 120 * 100 });

    odr::Road exportRoad = (odr::Road)road;
    test_map.id_to_road.insert({ exportRoad.id, exportRoad });
    test_map.export_file("C:\\Users\\guota\\Downloads\\test_left_turn_lane_1.xodr");


    //QApplication app(argc, argv);
    //MyGraphicsScene scene;
    //scene.DrawXodr(test_map);
    //QGraphicsView view(&scene);
    //view.scale(15, 15);
    //view.show();

    // return app.exec();
}