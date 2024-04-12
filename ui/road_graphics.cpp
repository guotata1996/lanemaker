#include "road_graphics.h"
#include "mainwindow.h"


#include <QGraphicsSceneMouseEvent>
#include <qgraphicsscene.h>
#include <qvector2d.h>

#include "spdlog/spdlog.h"
#include "stats.h"

extern QGraphicsScene* g_scene;

extern std::weak_ptr<RoadRunner::Road> g_PointerRoad;
extern double g_PointerRoadS;

namespace RoadRunner
{
    RoadGraphics::RoadGraphics(std::shared_ptr<RoadRunner::Road> _road,
        const odr::LaneSection& laneSection,
        double s_begin, double s_end) : 
        sBegin(s_begin), sEnd(s_end), Length(std::abs(s_begin - s_end)),
        road(_road)
    {
        g_scene->addItem(this);
        Create(laneSection);
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

    void RoadGraphics::Create(const odr::LaneSection& laneSection)
    {
        odr::Road& gen = road.lock()->generated;
        const double sMin = std::min(sBegin, sEnd);
        const double sMax = std::max(sBegin, sEnd);
        for (const auto& id2Lane : laneSection.id_to_lane)
        {
            const auto& lane = id2Lane.second;
            if (lane.type == "median" || lane.type == "driving")
            {
                odr::Line3D innerBorder, outerBorder;
                gen.get_lane_border_line(lane, sMin, sMax, 0.1f, outerBorder, innerBorder);
                auto aggregateBorder = outerBorder;
                aggregateBorder.insert(aggregateBorder.end(), innerBorder.rbegin(), innerBorder.rend());
                QPolygonF poly = LineToPoly(aggregateBorder);

                auto laneSegmentItem = new LaneSegmentGraphics(poly, outerBorder, innerBorder,
                    lane.type, this);

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
                            lines.push_back(gen.get_lane_marking_line(lane, sMin, sMax, refInner, refOffset, marking.width, 0.1f));
                            colors.push_back(markingGroup.color);
                        }
                        else if (markingGroup.type == "broken")
                        {
                            int nMarkingsPast = std::floor(sMin / (BrokenGap + BrokenLength));
                            double nextMarkingBegin = nMarkingsPast * (BrokenGap + BrokenLength);
                            for (double s = nextMarkingBegin; s <= sEnd; s += BrokenGap + BrokenLength)
                            {
                                double sBeginInSegment = std::max(s, sMin);
                                double sEndInSegment = std::min(s + BrokenLength, sEnd);
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
        std::string laneType,
        QGraphicsItem* parent) :
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
            subdivisionPortion.push_back(outerCumLength);
            outerCumLength += odr::euclDistance(outerP1, outerP2);
        }
        subdivisionPortion.push_back(outerCumLength);

        for (int i = 0; i != subdivisionPortion.size(); ++i)
        {
            subdivisionPortion[i] /= outerCumLength;
        }
        Stats::Instance("LaneSegmentGraphics Created")->Increment();
    }

    void LaneSegmentGraphics::mousePressEvent(QGraphicsSceneMouseEvent* evt)
    {
        QGraphicsPolygonItem::mousePressEvent(evt);

        spdlog::info("Mouse press evt on GraphicsItem @ {},{}", evt->pos().x(), evt->pos().y());
    }

    bool LaneSegmentGraphics::SnapCursor(QPointF scenePos)
    {
        auto parentRoad = dynamic_cast<RoadRunner::RoadGraphics*>(parentItem());
        double sBegin = parentRoad->sBegin;
        double sEnd = parentRoad->sEnd;

        QVector2D pEvent(scenePos);

        for (int i = 0; i != subdivisionPolys.size(); ++i)
        {
            const QPolygonF& subdivision = subdivisionPolys[i];
            if (subdivision.containsPoint(scenePos, Qt::FillRule::OddEvenFill))
            {
                double pMin = subdivisionPortion[i];
                double pMax = subdivisionPortion[i + 1];
                QVector2D p0(subdivision.at(0));
                QVector2D p1(subdivision.at(1));
                QVector2D p2(subdivision.at(2));
                QVector2D p3(subdivision.at(3));

                double dUp = pEvent.distanceToLine(p1, (p2 - p1).normalized());
                double dDown = pEvent.distanceToLine(p0, (p3 - p0).normalized());
                double portion = dDown / (dDown + dUp);
                portion = pMin * (1 - portion) + pMax * portion;
                double s = sBegin * (1 - portion) + sEnd * portion;
                s = odr::clamp(s, sBegin, sEnd);

                g_PointerRoad = parentRoad->road;
                g_PointerRoadS = s;
                return true;
            }
        }

        return false;
    }

    std::shared_ptr<Road> LaneSegmentGraphics::Road() const
    {
        auto parentRoad = dynamic_cast<RoadRunner::RoadGraphics*>(parentItem());
        return parentRoad->road.lock();
    }
}

