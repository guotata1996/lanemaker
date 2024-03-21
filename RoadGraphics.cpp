#include "RoadGraphics.h"
#include <QGraphicsSceneMouseEvent>

#include "mainwindow.h"
#include <qgraphicsscene.h>

#include "spdlog/spdlog.h"

extern QGraphicsScene* g_scene;

namespace RoadRunner
{
    RoadGraphics::RoadGraphics(RoadRunner::Road* _road) : road(_road)
    {
        g_scene->addItem(this);
    }

    RoadGraphics::~RoadGraphics()
    {
        g_scene->removeItem(this);
    }

    QPolygonF RoadGraphics::LineToPoly(const odr::Line3D& line)
    {
        QPolygonF rtn;
        for each (const odr::Vec3D & p in line)
        {
            rtn.append(QPointF(p[0], p[1]));
        }
        return rtn;
    }

    void RoadGraphics::Update(const odr::LaneSection& section, double s_begin, double s_end)
    {
        odr::Road& gen = road->generated;
        for (int side : {-1, 1})
        {
            for (auto& lane : section.get_sorted_driving_lanes(side))
            {
                auto innerBorder = gen.get_lane_border_line(lane, s_begin, s_end, 0.1f, false);
                auto outerBorder = gen.get_lane_border_line(lane, s_begin, s_end, 0.1f, true);
                outerBorder.insert(outerBorder.end(), innerBorder.rbegin(), innerBorder.rend());
                QPolygonF poly = LineToPoly(outerBorder);

                auto polyItem = new LaneSegmentGraphics(poly, this);
            }
        }
    }

    LaneSegmentGraphics::LaneSegmentGraphics(const QPolygonF poly, QGraphicsItem* parent) :
        QGraphicsPolygonItem(poly, parent) 
    {
        setAcceptHoverEvents(true);
    }

    void LaneSegmentGraphics::mousePressEvent(QGraphicsSceneMouseEvent* evt)
    {
        QGraphicsPolygonItem::mousePressEvent(evt);

        spdlog::info("Mouse press evt on GraphicsItem @ {},{}", evt->pos().x(), evt->pos().y());
    }

    void LaneSegmentGraphics::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
    {
        auto parentRoad = dynamic_cast<RoadRunner::RoadGraphics*>(parentItem());
        spdlog::info("Enter road {}", parentRoad->road->ID());

        // TODO: figure out s
        // TODO: highlight
    }
}

