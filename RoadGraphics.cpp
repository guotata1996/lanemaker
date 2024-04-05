#include "RoadGraphics.h"
#include "mainwindow.h"


#include <QGraphicsSceneMouseEvent>
#include <qgraphicsscene.h>
#include <qvector2d.h>

#include "spdlog/spdlog.h"

extern QGraphicsScene* g_scene;

extern std::weak_ptr<RoadRunner::Road> g_PointerRoad;
extern double g_PointerRoadS;

namespace RoadRunner
{
    RoadGraphics::RoadGraphics(std::shared_ptr<RoadRunner::Road> _road) : road(_road)
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
        odr::Road& gen = road.lock()->generated;

        for (const auto& id2Lane : section.id_to_lane)
        {
            const auto& lane = id2Lane.second;
            if (lane.type == "median" || lane.type == "driving")
            {
                odr::Line3D innerBorder, outerBorder;
                gen.get_lane_border_line(lane, s_begin, s_end, 0.1f, outerBorder, innerBorder);
                auto aggregateBorder = outerBorder;
                aggregateBorder.insert(aggregateBorder.end(), innerBorder.rbegin(), innerBorder.rend());
                QPolygonF poly = LineToPoly(aggregateBorder);

                auto laneSegmentItem = new LaneSegmentGraphics(poly, outerBorder, innerBorder,
                    s_begin, s_end, road.lock()->ID(), lane.type, this);

                for (const auto& markingGroup : lane.roadmark_groups)
                {
                    for (const auto& marking : markingGroup.roadmark_lines)
                    {
                        std::vector<odr::Line3D> lines;
                        std::vector<std::string> colors;
                        bool refInner = std::abs(marking.t_offset) < RoadRunner::LaneWidth / 2;
                        double refOffset = marking.t_offset;
                        if (!refInner)
                        {
                            refOffset += lane.id < 0 ? RoadRunner::LaneWidth : -RoadRunner::LaneWidth;
                        }
                        if (markingGroup.type == "solid")
                        {
                            lines.push_back(gen.get_lane_marking_line(lane, s_begin, s_end, refInner, refOffset, marking.width, 0.1f));
                            colors.push_back(markingGroup.color);
                        }
                        else if (markingGroup.type == "broken")
                        {
                            int nMarkingsPast = std::floor(s_begin / (BrokenGap + BrokenLength));
                            double nextMarkingBegin = nMarkingsPast * (BrokenGap + BrokenLength);
                            for (double s = nextMarkingBegin; s <= s_end; s += BrokenGap + BrokenLength)
                            {
                                double sBeginInSegment = std::max(s, s_begin);
                                double sEndInSegment = std::min(s + BrokenLength, s_end);
                                if (sEndInSegment > sBeginInSegment + 0.1f)
                                {
                                    odr::Line3D markingLine = gen.get_lane_marking_line(lane, 
                                        sBeginInSegment, sEndInSegment, refInner, refOffset, marking.width, 0.1f);
                                    lines.push_back(markingLine);
                                    colors.push_back(markingGroup.color);
                                }
                            }
                        }

                        for (int i = 0; i != lines.size(); ++i)
                        {
                            QPolygonF markingPoly = LineToPoly(lines[i]);
                            auto markingItem = new QGraphicsPolygonItem(markingPoly, this);
                            markingItem->setZValue(1);
                            markingItem->setPen(Qt::NoPen);
                            Qt::GlobalColor color = colors[i] == "yellow" ? Qt::yellow : Qt::white;
                            markingItem->setBrush(QBrush(color, Qt::SolidPattern));
                        }
                    }
                }
            }
        }
    }

    LaneSegmentGraphics::LaneSegmentGraphics(
        const QPolygonF& poly,
        odr::Line3D outerBorder, 
        odr::Line3D innerBorder,
        double aSBegin, double aSEnd, std::string aRoadID, std::string laneType,
        QGraphicsItem* parent) :
        sBegin(aSBegin), sEnd(aSEnd), roadID(aRoadID),
        QGraphicsPolygonItem(poly, parent) 
    {
        setAcceptHoverEvents(true);

        setPen(Qt::NoPen);
        setBrush(QBrush(laneType == "median" ? Qt::yellow : Qt::gray, Qt::SolidPattern));
        
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

    void LaneSegmentGraphics::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
    {
        hoverMoveEvent(event);
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
                g_PointerRoad = parentRoad->road;
                g_PointerRoadS = s;
                break;
            }
        }
    }

    void LaneSegmentGraphics::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
    {
        auto g_road = g_PointerRoad.lock();
        if (g_road != nullptr && g_road->ID() == roadID &&
            sBegin <= g_PointerRoadS && g_PointerRoadS <= sEnd)
        {
            g_PointerRoad.reset();
        }
    }
}

