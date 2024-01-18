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
    road.SetLength(90 * 100);
    RoadRunnder::LaneSection rs1{ RoadRunnder::RoadProfile{0, 0}, 0 * 100 };
    //RoadRunnder::LaneSection rs2{ RoadRunnder::RoadProfile{0, 1}, 45 * 100 };
    road.AddRightSection(rs1); // Even if there's no lane on the right, must have a right section. Need to fix. 
    //road.AddRightSection(rs2);

    RoadRunnder::LaneSection ls1{ RoadRunnder::RoadProfile{1, 1}, 0 * 100 };
    RoadRunnder::LaneSection ls2{ RoadRunnder::RoadProfile{0, 1}, 45 * 100 };
    road.AddLeftSection(ls1);
    road.AddLeftSection(ls2);

    odr::Road exportRoad = (odr::Road)road;

    test_map.id_to_road.insert({ exportRoad.id, exportRoad });
    test_map.export_file("C:\\Users\\guota\\Downloads\\export.xodr");


    //QApplication app(argc, argv);
    //MyGraphicsScene scene;
    //scene.DrawXodr(test_map);
    //QGraphicsView view(&scene);
    //view.scale(15, 15);
    //view.show();

    // return app.exec();
}