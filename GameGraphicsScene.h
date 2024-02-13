#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_2.h>

#include "OpenDriveMap.h"
#include <spdlog/spdlog.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_2<K>                            Triangulation;
typedef Triangulation::Point                                Point;

class MyGraphicsScene : public QGraphicsScene
{
public:
    void DrawXodr(odr::OpenDriveMap& odr_map)
    {
        QBrush solid;
        solid.setColor(Qt::darkGreen);
        solid.setStyle(Qt::SolidPattern);
        QPen noPen;
        noPen.setColor(Qt::transparent);
        noPen.setWidth(0);

        for each (odr::Junction junction in odr_map.get_junctions())
        {
            auto conns = junction.id_to_connection;
            for (auto it = conns.begin(); it != conns.end(); it++)
            {
                std::string conn_road_id = it->second.connecting_road;
                odr::Road conn_road = odr_map.id_to_road.at(conn_road_id);
                DrawRoad(conn_road, noPen, solid);

            }
        }

        for each (odr::Road road in odr_map.get_roads())
        {
            if (road.junction == "-1")
            {
                DrawRoad(road);
            }
        }
    }

private:

    QPolygonF LineToPoly(const odr::Line3D& line)
    {
        QPolygonF rtn;
        for each (const odr::Vec3D& p in line)
        {
            rtn.append(QPointF(p[0], p[1]));
        }
        return rtn;
    }

    void DrawRoad(const odr::Road& road, QPen pen=QPen(), QBrush brush=QBrush())
    {
        auto s_to_lane_section = road.s_to_lanesection;
        for (auto it2 = s_to_lane_section.begin(); it2 != s_to_lane_section.end(); it2++)
        {
            for each (odr::Lane lane in it2->second.get_lanes())
            {
                if (lane.type == "driving")
                {
                    odr::Line3D outerBorder = road.get_lane_border_line(lane, 0.1f, true);
                    odr::Line3D innerBorder = road.get_lane_border_line(lane, 0.1f, false);
                    outerBorder.insert(outerBorder.end(), innerBorder.rbegin(), innerBorder.rend());

                    QPolygonF poly = LineToPoly(outerBorder);
                    this->addPolygon(poly, pen, brush);
                }
            }
        }
    }

//    virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent* mouseEvent)
//    {
//        QPointF pos = mouseEvent->scenePos();
//        m_points.push_back(Point(pos.x(), pos.y()));
//
//        if (m_points.size() >= 1)
//        {
//            Triangulation t;
//            t.insert(m_points.begin(), m_points.end());
//
//            this->clear();
//            for (auto it = t.edges_begin(); it != t.edges_end(); ++it)
//            {
//                auto p1 = t.segment(it).source();
//                auto p2 = t.segment(it).target();
//
//                this->addLine(p1.x(), p1.y(), p2.x(), p2.y(), QPen(QColor::fromRgb(255, 0, 128)));
//            }
//        }
//    }
//
//    std::vector <Point> m_points;

};