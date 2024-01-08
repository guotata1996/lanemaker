#include <fstream>


#include <QApplication>
#include <QPushButton>
#include <QGraphicsView>

#include <string>

#include "Geometries/Line.h"

#include "GameGraphicsScene.h"


int main(int argc, char** argv)
{
    /*
    odr::OpenDriveMap odr_map("C:\\Users\\guota\\Downloads\\Town06.xodr");
    odr_map.export_file("C:\\Users\\guota\\Downloads\\export.xodr");

    QApplication app(argc, argv);
    MyGraphicsScene scene;
    scene.DrawXodr(odr_map);
    QGraphicsView view(&scene);
    view.scale(5, 5);
    view.show();

    return app.exec();
    */

    
    odr::OpenDriveMap test_map;
    odr::Road road("road1", 75, "1", "road 1");

    // geometry
    road.ref_line.s0_to_geometry[0] = std::make_unique<odr::Line>(0, 0, 0, 0, 100);

    // lane offset
    odr::Poly3 zero(0, 0, 0, 0, 0);
    road.lane_offset.s0_to_poly.insert({ 0, zero });
    
    // Lane section 1
    odr::LaneSection section1("1", 0);

    odr::Lane center("1", 0, 0, false, "");
    odr::RoadMarkGroup g("1", 0, 0, 0.12, 0, 0, "broken", "standard", "standard", "standard", "both");
    odr::RoadMarksLine ml("1", 0, 0, 0, 0.12, 3, 6, 0, 0, "", "");
    g.roadmark_lines.insert(ml);
    center.roadmark_groups.insert(g);

    odr::Lane r("-1", 0, -1, false, "driving");
    odr::Poly3 constWidth(0, 3.25, 0, 0, 0);
    r.lane_width.s0_to_poly.insert({ 0, constWidth });

    section1.id_to_lane.insert({ 0, center });
    section1.id_to_lane.insert({ -1, r });

    road.s_to_lanesection.insert({ 0, section1 });

    // Lane section 2: narrow
    odr::LaneSection section2("2", 0);
    if (false)
    {
        
        odr::Lane r1("-1", 0, -1, false, "driving");
        odr::Poly3 widthNarrowing(0, 3.25, 0, -3.9e-3, 5.2e-5);
        r1.lane_width.s0_to_poly.insert({ 25, widthNarrowing }); // FIX: use 0

        section2.id_to_lane.insert({ 0, center });
        section2.id_to_lane.insert({ -1, r1 });
    }
    else
    {
        section2.id_to_lane.insert({ 0, center });

        odr::Lane r1("-1", 0, -1, false, "driving");
        r1.lane_width.s0_to_poly.insert({ 25, constWidth });
        section2.id_to_lane.insert({ -1, r1 });

        odr::Lane r2("-2", 0, -2, false, "driving");
        odr::Poly3 widthWidening(0, 0, 0, 3.9e-3, -5.2e-5);
        r2.lane_width.s0_to_poly.insert({ 25, widthWidening });
        section2.id_to_lane.insert({ -2, r2 });
    }

    road.s_to_lanesection.insert({ 25, section2 });

    test_map.id_to_road.insert({ "road1", road });
    test_map.export_file("C:\\Users\\guota\\Downloads\\export.xodr");
}