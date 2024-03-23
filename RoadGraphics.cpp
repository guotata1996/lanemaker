#include "RoadGraphics.h"
#include "mainwindow.h"


#include <QGraphicsSceneMouseEvent>
#include <qgraphicsscene.h>
#include <qvector2d.h>

#include "spdlog/spdlog.h"

extern QGraphicsScene* g_scene;

extern std::string g_PointerRoadID;
extern double g_PointerRoadS;

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
        for (const odr::Vec3D & p : line)
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
                auto aggregateBorder = outerBorder;
                aggregateBorder.insert(aggregateBorder.end(), innerBorder.rbegin(), innerBorder.rend());
                QPolygonF poly = LineToPoly(aggregateBorder);

                auto polyItem = new LaneSegmentGraphics(poly, outerBorder, innerBorder, 
                    s_begin, s_end, road->ID(), this);
            }
        }
    }

    LaneSegmentGraphics::LaneSegmentGraphics(
        const QPolygonF& poly,
        const odr::Line3D& outerBorder, 
        const odr::Line3D& innerBorder,
        double aSBegin, double aSEnd, std::string aRoadID,
        QGraphicsItem* parent) :
        sBegin(aSBegin), sEnd(aSEnd), roadID(aRoadID),
        QGraphicsPolygonItem(poly, parent) 
    {
        setAcceptHoverEvents(true);
        QPen noPen;
        noPen.setWidth(0);
        setPen(noPen);
        setBrush(QBrush(Qt::gray, Qt::SolidPattern));
        
        assert(outerBorder.size() == innerBorder.size());
        assert(outerBorder.size() >= 2);

        double outerCumLength = 0;
        for (int i = 0; i < outerBorder.size() - 1; ++i)
        {
            auto outerP1 = outerBorder[i];
            auto outerP2 = outerBorder[i + 1];
            auto innerP1 = innerBorder[i];
            auto innerP2 = innerBorder[i + 1];
            odr::Line3D subdivision;
            subdivision.push_back(outerP1);
            subdivision.push_back(outerP2);
            subdivision.push_back(innerP2);
            subdivision.push_back(innerP1);
            subdivisionPolys.push_back(RoadGraphics::LineToPoly(subdivision));
            subdivisionS.push_back(outerCumLength);
            outerCumLength += odr::euclDistance(outerP1, outerP2);
        }
        subdivisionS.push_back(outerCumLength);

        double correctionFactor = (sEnd - sBegin) / outerCumLength;
        for (int i = 0; i != subdivisionS.size(); ++i)
        {
            subdivisionS[i] = sBegin + correctionFactor * subdivisionS[i];
        }
    }

    void LaneSegmentGraphics::mousePressEvent(QGraphicsSceneMouseEvent* evt)
    {
        QGraphicsPolygonItem::mousePressEvent(evt);

        spdlog::info("Mouse press evt on GraphicsItem @ {},{}", evt->pos().x(), evt->pos().y());
    }

    void LaneSegmentGraphics::hoverMoveEvent(QGraphicsSceneHoverEvent* event)
    {
        auto parentRoad = dynamic_cast<RoadRunner::RoadGraphics*>(parentItem());

        auto eventPos = event->scenePos();
        QVector2D pEvent(eventPos);

        for (int i = 0; i != subdivisionPolys.size(); ++i)
        {
            const QPolygonF& subdivision = subdivisionPolys[i];
            if (subdivision.containsPoint(eventPos, Qt::FillRule::OddEvenFill))
            {
                double sMin = subdivisionS[i];
                double sMax = subdivisionS[i + 1];
                QVector2D p0(subdivision.at(0));
                QVector2D p1(subdivision.at(1));
                QVector2D p2(subdivision.at(2));
                QVector2D p3(subdivision.at(3));

                double dUp = pEvent.distanceToLine(p1, (p2 - p1).normalized());
                double dDown = pEvent.distanceToLine(p0, (p3 - p0).normalized());
                double portion = dDown / (dDown + dUp);
                double s = sMin * (1 - portion) + sMax * portion;
                s = std::max(sBegin, std::min(s, sEnd));
                g_PointerRoadID = parentRoad->road->ID();
                g_PointerRoadS = s;
                break;
            }
        }
    }

    void LaneSegmentGraphics::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
    {
        if (g_PointerRoadID == roadID && 
            sBegin <= g_PointerRoadS && g_PointerRoadS <= sEnd)
        {
            g_PointerRoadID = "";
        }
    }
}

