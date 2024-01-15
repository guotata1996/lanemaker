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
    /*
    odr::Road road("road1", 150, "-1", "road 1");

    // geometry
    road.ref_line.s0_to_geometry[0] = std::make_unique<odr::Line>(0, 0, 0, 0, 100);

    // lane offset
    odr::Poly3 zero(0, 0, 0, 0, 0);
    road.lane_offset.s0_to_poly.insert({ 0, zero });
    
    // Lane section 1
    odr::LaneSection section1("1", 0);
    {
        odr::Lane center("1", 0, 0, false, "");
        //odr::RoadMarkGroup g("1", 0, 0, 0.12, 0, 0, "broken", "standard", "standard", "standard", "both");
        //odr::RoadMarksLine ml("1", 0, 0, 0, 0.12, 3, 6, 0, 0, "", "");
        //g.roadmark_lines.insert(ml);
        //center.roadmark_groups.insert(g);

        odr::Lane r("1", 0, -1, false, "driving");
        odr::Poly3 constWidth(0, 3.25, 0, 0, 0);
        r.lane_width.s0_to_poly.insert({ 0, constWidth });

        section1.id_to_lane.insert({ 0, center });
        section1.id_to_lane.insert({ -1, r });
        
    }
    test_map.derive_lane_borders(road, section1);
    road.s_to_lanesection.insert({ 0, section1 });

    // Lane section 2: widen
    odr::LaneSection section2("1", 25);
    {
        odr::Lane center("1", 25, 0, false, "");
        section2.id_to_lane.insert({ 0, center });

        odr::Lane r1("-1", 25, -1, false, "driving");
        odr::Poly3 constWidth(25, 3.25, 0, 0, 0);
        r1.lane_width.s0_to_poly.insert({ 25, constWidth });
        section2.id_to_lane.insert({ -1, r1 });

        odr::Lane r2("-2", 25, -2, false, "driving");
        odr::Poly3 widthWidening(25, 0, 0, 3.9e-3, -5.2e-5);
        r2.lane_width.s0_to_poly.insert({ 25, widthWidening });
        section2.id_to_lane.insert({ -2, r2 });
    }
    test_map.derive_lane_borders(road, section2);
    road.s_to_lanesection.insert({ 25, section2 });

    // Lane section 3: keep then narrow
    odr::LaneSection section3("1", 75);
    {
        odr::Lane center("1", 75, 0, false, "");
        section3.id_to_lane.insert({ 0, center });

        odr::Lane r1("-1", 75, -1, false, "driving");
        odr::Poly3 constWidth(75, 3.25, 0, 0, 0);
        r1.lane_width.s0_to_poly.insert({ 75, constWidth });
        section3.id_to_lane.insert({ -1, r1 });

        odr::Lane r2("-2", 75, -2, false, "driving");
        r2.lane_width.s0_to_poly.insert({ 75, constWidth });
        odr::Poly3 widthNarrowing(100, 3.25, 0, -3.9e-3, 5.2e-5);
        r2.lane_width.s0_to_poly.insert({ 100, widthNarrowing });
        section3.id_to_lane.insert({ -2, r2 });
    }
    test_map.derive_lane_borders(road, section3);
    road.s_to_lanesection.insert({ 75, section3 });
    */

    RoadRunnder::Road road;
    RoadRunnder::LaneSection s1{ RoadRunnder::RoadProfile{0, 0, 2, 1}, 30 * 100 };
    RoadRunnder::LaneSection s2{ RoadRunnder::RoadProfile{0, 0, 0, 1}, 60 * 100 };
    road.AddSection(s1);
    road.AddSection(s2);

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