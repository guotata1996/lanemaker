#include "road_graphics.h"

#include <QGraphicsSceneMouseEvent>
#include <qgraphicsscene.h>
#include <qvector2d.h>
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <math.h>

#include "spdlog/spdlog.h"
#include "stats.h"
#include "junction.h"
#include "map_view_gl.h"

extern QGraphicsScene* g_scene;

namespace RoadRunner
{
    SectionGraphics::SectionGraphics(std::shared_ptr<RoadRunner::Road> road,
        const odr::LaneSection& laneSection,
        double sBegin, double sEnd)
    {
        //g_scene->addItem(this);
        auto Length = (std::abs(sBegin - sEnd));
        
        odr::Road& gen = road->generated;
        bool biDirRoad = gen.rr_profile.HasSide(-1) && gen.rr_profile.HasSide(1);
        const double sMin = std::min(sBegin, sEnd);
        const double sMax = std::max(sBegin, sEnd);
        sectionElevation = gen.ref_line.elevation_profile.get((sMin + sMax) / 2);
        setZValue(sectionElevation);
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
                const int laneID = id2Lane.first;
                int laneIDWhenReversed = 0;
                if (biDirRoad)
                {
                    if (lane.type == "median")
                    {
                        assert(laneID == 1);
                        laneIDWhenReversed = 1;
                    }
                    else
                    {
                        laneIDWhenReversed = -laneID + 1;
                    }
                }
                else
                {
                    laneIDWhenReversed = -laneID;
                }
                auto laneSegmentItem = new LaneGraphics(poly, outerBorder, innerBorder,
                    laneID, laneIDWhenReversed, lane.type, this);
                allLaneGraphics.push_back(laneSegmentItem);

                // TODO: subdivide to more faces
                // TODO: re-index moved & reversed segments
                allSpatialIndice.push_back(SpatialIndexer::Instance()->Index(gen, lane, sMin, sMax));
                allGraphicsIndice.push_back(g_mapViewGL->AddQuads(innerBorder, outerBorder));
            }
        }

        // Draw road markings
        for (const auto& id2Lane : laneSection.id_to_lane)
        {
            const auto& lane = id2Lane.second;
            for (auto groupIt = lane.roadmark_groups.begin(); groupIt != lane.roadmark_groups.end(); ++groupIt)
            {
                auto nextGroupIt = groupIt;
                nextGroupIt++;
                double groupsBegin = std::max(sMin, laneSection.s0 + groupIt->s_offset);
                double groupsEnd = nextGroupIt == lane.roadmark_groups.end() ? sMax : std::min(sMax, laneSection.s0 + nextGroupIt->s_offset);

                if (groupsBegin >= groupsEnd)
                {
                    // road marking group doesn't belong to this segment grapghics
                    continue;
                }

                std::vector<odr::Line3D> lines;
                std::vector<std::string> colors;

                if (groupIt->type == "solid" || groupIt->type == "curb")
                {
                    lines.push_back(gen.get_lane_marking_line(lane, groupsBegin, groupsEnd, groupIt->width, 0.1f));
                    colors.push_back(groupIt->color);
                }
                else if (groupIt->type == "broken")
                {
                    int nMarkingsPast = std::floor(groupsBegin / (BrokenGap + BrokenLength));
                    double nextMarkingBegin = nMarkingsPast * (BrokenGap + BrokenLength);
                    for (double s = nextMarkingBegin; s <= groupsEnd; s += BrokenGap + BrokenLength)
                    {
                        double sBeginInSegment = std::max(s, groupsBegin);
                        double sEndInSegment = std::min(s + BrokenLength, groupsEnd);
                        if (sEndInSegment > sBeginInSegment + 0.1f)
                        {
                            odr::Line3D markingLine = gen.get_lane_marking_line(lane,
                                sBeginInSegment, sEndInSegment, groupIt->width, 0.1f);
                            lines.push_back(markingLine);
                            colors.push_back(groupIt->color);
                        }
                    }
                }
                else
                {
                    continue;
                }

                for (int i = 0; i != lines.size(); ++i)
                {
                    QPolygonF markingPoly = LineToPoly(lines[i]);
                    auto markingItem = new MarkingGraphics(markingPoly, this);
                    markingItem->setPen(Qt::NoPen);
                    Qt::GlobalColor color = colors[i] == "yellow" ? Qt::yellow :
                        (colors[i] == "white" ? Qt::white : Qt::lightGray);
                    markingItem->setBrush(QBrush(color, Qt::SolidPattern));
                }
            }
        }

        // Draw road objects if needed
        for (const auto& id_object : gen.id_to_object)
        {
            if (sMin <= id_object.second.s0 && id_object.second.s0 < sMax)
            {
                if (id_object.second.type == "roadMark")
                {
                    QAbstractGraphicsShapeItem* markingItem;
                    if (id_object.second.subtype == "stopping-line")
                    {
                        auto s = id_object.second.s0;
                        auto w = id_object.second.width;
                        auto s1 = s - w / 2;
                        auto s2 = s + w / 2;
                        auto polys = gen.get_both_dirs_poly(s1, s2, 0.1);
                        auto poly = id_object.first == std::to_string(odr::RoadLink::ContactPoint_Start) ? polys.first : polys.second;
                        QPolygonF markingPoly = LineToPoly(poly);
                        markingItem = new QGraphicsPolygonItem(markingPoly, this);
                        markingItem->setPen(Qt::NoPen);
                        Qt::GlobalColor color = Qt::white;
                        markingItem->setBrush(QBrush(color, Qt::SolidPattern));
                    }
                    else if (id_object.second.subtype == "arrow")
                    {
                        auto sMid = (sMin + sMax) / 2;  // arrow always placed at mid at current section
                        auto pt = gen.get_surface_pt(sMid, id_object.second.t0);
                        auto hdg = gen.ref_line.get_hdg(sMid);
                        int arrowType = std::stoi(id_object.second.name);
                        markingItem = new ArrowGraphics(arrowType, this);
                        QTransform arrowTransform;
                        arrowTransform.translate(pt[0], pt[1]);
                        arrowTransform.rotate(180 / M_PI * id_object.second.hdg);
                        markingItem->setTransform(arrowTransform);
                    }
                    else
                    {
                        spdlog::warn("roadMark subtype {} isn't supported!", id_object.second.subtype);
                        continue;
                    }
                    markingItem->setPen(Qt::NoPen);
                    Qt::GlobalColor color = Qt::white;
                    markingItem->setBrush(QBrush(color, Qt::SolidPattern));
                }
            }
        }

        refLineHint = new QGraphicsPathItem(this);
        refLineHint->setPen(QPen(Qt::green, 0.3, Qt::SolidLine));
        refLineHint->hide();
        auto lineAppox = gen.ref_line.get_line(std::min(sBegin, sEnd), std::max(sBegin, sEnd), 0.1f);
        refLinePath = CreateRefLinePath(lineAppox);
        std::reverse(lineAppox.begin(), lineAppox.end());
        refLinePathReversed = CreateRefLinePath(lineAppox);
        refLineHint->setPath(refLinePath);
    }

    SectionGraphics::~SectionGraphics()
    {
        //g_scene->removeItem(this);
        if (!Road::ClearingMap)
        {
            for (auto index : allSpatialIndice)
            {
                SpatialIndexer::Instance()->UnIndex(index);
            }
        }
        for (auto index : allGraphicsIndice)
        {
            g_mapViewGL->RemoveItem(index);
        }
    }

    void SectionGraphics::EnableHighlight(bool enabled, bool bringToTop)
    {
        double liftedElevation = bringToTop ? 128 : sectionElevation + 0.01;
        setZValue(enabled ? liftedElevation : sectionElevation);
        for (auto laneSegment : allLaneGraphics)
        {
            if (laneSegment != nullptr)
            {
                laneSegment->EnableHighlight(enabled);
            }
        }
        refLineHint->setVisible(enabled);
    }

    QPainterPath SectionGraphics::CreateRefLinePath(const odr::Line3D& lineAppox)
    {
        QPainterPath refLinePath;

        if (lineAppox.size() >= 2)
        {
            // Ref line
            auto initial = lineAppox[0];

            refLinePath.moveTo(initial[0], initial[1]);
            for (int i = 1; i < lineAppox.size(); ++i)
            {
                auto p = lineAppox[i];
                refLinePath.lineTo(p[0], p[1]);
            }
            // Arrow
            const auto& last = lineAppox.back();
            const auto& last2 = lineAppox[lineAppox.size() - 2];
            QVector2D lastDir(last[0] - last2[0], last[1] - last2[1]);
            lastDir.normalize();
            QVector2D arrowHead(last[0], last[1]);
            QVector2D arrowTail = arrowHead - lastDir * 1;
            QVector2D arrowLeftDir(-lastDir.y(), lastDir.x());
            QVector2D arrowLeft = arrowTail + arrowLeftDir * 1;
            QVector2D arrowRight = arrowTail - arrowLeftDir * 1;
            refLinePath.moveTo(arrowLeft.toPointF());
            refLinePath.lineTo(arrowHead.toPointF());
            refLinePath.lineTo(arrowRight.toPointF());
        }
        return refLinePath;
    }

    void SectionGraphics::updateIndexingInfo(std::string newRoadID, double sBegin, double sEnd)
    {
        for (auto index : allSpatialIndice)
        {
            uint32_t face1ID = index >> 32;
            uint32_t face2ID = index & 0xffffffff;
            for (auto faceID : { face1ID , face2ID })
            {
                if (faceID == SpatialIndexer::InvalidFace) continue;
                Quad& face = SpatialIndexer::Instance()->faceInfo.at(faceID);
                face.roadID = newRoadID;
                face.sBegin = sBegin;
                face.sEnd = sEnd;
            }

        }
        refLineHint->setPath(sBegin < sEnd ? refLinePath : refLinePathReversed);
    }

    double SectionGraphics::SBegin() const
    {
        return SpatialIndexer::Instance()->faceInfo.at(allSpatialIndice.front()).sBegin;
    }

    double SectionGraphics::SEnd() const
    {
        return SpatialIndexer::Instance()->faceInfo.at(allSpatialIndice.front()).sEnd;
    }

    double SectionGraphics::Length() const
    {
        return std::abs(SBegin() - SEnd());
    }

    LaneGraphics::LaneGraphics(
        const QPolygonF& poly,
        odr::Line3D outerBorder, 
        odr::Line3D innerBorder,
        int laneID, int laneIDRev,
        std::string laneType,
        QGraphicsItem* parent) :
        QGraphicsPolygonItem(poly, parent),
        HighlightColor(189, 187, 185),
        laneID(laneID), laneIDReversed(laneIDRev),
        isMedian(laneType == "median")
    {
        setAcceptHoverEvents(true);

        setPen(Qt::NoPen);
        EnableHighlight(false);
        
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
            subdivisionPolys.push_back(LineToPoly(subdivision));
            subdivisionPortion.push_back(outerCumLength);
            outerCumLength += odr::euclDistance(outerP1, outerP2);
        }
        subdivisionPortion.push_back(outerCumLength);

        for (int i = 0; i != subdivisionPortion.size(); ++i)
        {
            subdivisionPortion[i] /= outerCumLength;
        }

        odr::Line3D lowPolyV = { outerBorder.front(), outerBorder.back(), innerBorder.back(), innerBorder.front() };
        lowLODPoly = LineToPoly(lowPolyV);

        Stats::Instance("LaneGraphics Created")->Increment();
    }

    void LaneGraphics::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
    {
        const qreal lod = option->levelOfDetailFromTransform(painter->worldTransform());
        if (lod > 2)
        {
            QGraphicsPolygonItem::paint(painter, option, widget);
        }
        else
        {
            painter->save();
            painter->setPen(pen());
            painter->setBrush(brush());
            painter->drawPolygon(lowLODPoly);
            painter->restore();
        }
    }

    // TODO:
    std::shared_ptr<Road> LaneGraphics::GetRoad() const
    {
        auto parentRoad = dynamic_cast<RoadRunner::SectionGraphics*>(parentItem());
        return nullptr; // parentRoad->road.lock();
    }

    void LaneGraphics::EnableHighlight(bool enabled)
    {
        setBrush(QBrush(isMedian ? Qt::yellow : (enabled ? HighlightColor : Qt::darkGray), Qt::SolidPattern));
    }


    MarkingGraphics::MarkingGraphics(const QPolygonF& polygon, QGraphicsItem* parent) :
        QGraphicsPolygonItem(polygon, parent)
    {
        lowLODPoly << polygon.front();
        lowLODPoly << polygon.at(polygon.size() / 2 - 1);
        lowLODPoly << polygon.at(polygon.size() / 2);
        lowLODPoly << polygon.back();

    };

    void MarkingGraphics::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
    {
        const qreal lod = option->levelOfDetailFromTransform(painter->worldTransform());
        if (lod > 4)
        {
            QGraphicsPolygonItem::paint(painter, option, widget);
        }
        else
        {
            painter->save();
            painter->setPen(pen());
            painter->setBrush(brush());
            painter->drawPolygon(lowLODPoly);
            painter->restore();
        }
    }
    
    JunctionGraphics::JunctionGraphics(const odr::Line2D& boundary)
    {
        QPainterPath path;
        setPen(Qt::NoPen);
        setBrush(DefaultBrush);
        path.addPolygon(LineToPoly(boundary));
        setPath(path);
        //g_scene->addItem(this);
        setAcceptHoverEvents(true);
    }

    JunctionGraphics::JunctionGraphics(const std::vector<std::pair<odr::Line2D, odr::Line2D>>& boundary)
    {
        QPainterPath path;

        setPen(Qt::NoPen);
        setBrush(DefaultBrush);

        const int ZebraLineWidth = 8;
        const int ZebraLineSkip = 16;

        for (const auto& dualSides : boundary)
        {
            odr::Line2D singleBoundary = dualSides.first;
            if (singleBoundary.empty())
            {
                spdlog::trace("Empty single boundary passed into JunctionGraphics");
                continue;
            }
            auto pOrigin = singleBoundary.front();
            auto p1 = singleBoundary.back();
            auto p2 = dualSides.second.back();
            auto pM = StripMidPoint(pOrigin, p1, p2);
            singleBoundary.push_back(pM);

            singleBoundary.insert(singleBoundary.end(), dualSides.second.rbegin(), dualSides.second.rend());
            path.addPolygon(LineToPoly(singleBoundary));

            for (int i = 0; i < dualSides.first.size() - ZebraLineWidth; i += ZebraLineWidth + ZebraLineSkip)
            {
                auto p1 = dualSides.first.at(i);
                auto p2 = dualSides.second.at(i);
                if (odr::euclDistance(p1, p2) < 0.5) continue;
                auto pM1 = StripMidPoint(pOrigin, p1, p2);

                auto p3 = dualSides.first.at(i + ZebraLineWidth);
                auto p4 = dualSides.second.at(i + ZebraLineWidth);
                if (odr::euclDistance(p3, p4) < 0.5) continue;
                auto pM2 = StripMidPoint(pOrigin, p3, p4);

                QPolygonF singleStrip({ QPointF(p1[0], p1[1]), QPointF(pM1[0], pM1[1]), QPointF(p2[0], p2[1]), 
                    QPointF(p4[0], p4[1]), QPointF(pM2[0], pM2[1]), QPointF(p3[0], p3[1]) });
                auto line = new QGraphicsPolygonItem(singleStrip, this);
                line->setBrush(QBrush(Qt::lightGray));
                line->setPen(Qt::NoPen);
            }
        }
        
        setPath(path);
        //g_scene->addItem(this);
    }

    JunctionGraphics::~JunctionGraphics()
    {
        //g_scene->removeItem(this);
    }

    void JunctionGraphics::hoverEnterEvent(QGraphicsSceneHoverEvent* evt)
    {
        setBrush(Qt::NoBrush);
        junctionElevation = zValue();
        setZValue(128.1); // higher than highlighted linked road to properly receive hoverLeaveEvent
    }

    void JunctionGraphics::hoverLeaveEvent(QGraphicsSceneHoverEvent* evt)
    {
        setBrush(DefaultBrush);
        setZValue(junctionElevation);
    }

    odr::Vec2D JunctionGraphics::StripMidPoint(const odr::Vec2D& pOrigin, const odr::Vec2D& p1, const odr::Vec2D& p2)
    {
        auto pM = odr::mut(0.5, odr::add(p1, p2));
        auto p1p2 = odr::sub(p2, p1);
        auto pMOffset = odr::mut(0.2, odr::Vec2D{ -p1p2[1], p1p2[0] });
        auto p0p1 = odr::sub(p1, pOrigin);
        if (odr::dot(pMOffset, p0p1) > 0)
        {
            pMOffset = odr::negate(pMOffset);
        }
        return odr::add(pM, pMOffset);
    }

    ArrowGraphics::ArrowGraphics(int markingType, QGraphicsItem* parent):
        QGraphicsPathItem(parent)
    {
        QPainterPath path;
        path.setFillRule(Qt::WindingFill);
        if ((markingType & DeadEnd) != 0)
        {
            QPolygonF stem, cross1, cross2;
            stem << QPointF(-2, 0.2) << QPointF(1, 0.2) << QPointF(1, -0.2) << QPointF(-2, -0.2);
            path.addPolygon(stem);
            cross1 << QPointF(0.4, 1) << QPointF(0.6, 1) << QPointF(1.6, -1) << QPointF(1.4, -1);
            path.addPolygon(cross1);
            cross2 << QPointF(1.4, 1) << QPointF(1.6, 1) << QPointF(0.6, -1) << QPointF(0.4, -1);
            path.addPolygon(cross2);
        }
        if ((markingType & Turn_No) != 0)
        {
            QPolygonF shape;
            shape << QPointF(-2, 0.2);
            shape << QPointF(1, 0.2);
            shape << QPointF(1, 0.5);
            shape << QPointF(2, 0);
            shape << QPointF(1, -0.5);
            shape << QPointF(1, -0.2);
            shape << QPointF(-2, -0.2);
            path.addPolygon(shape);
        }
        if ((markingType & Turn_Left) != 0)
        {
            QPolygonF shape;
            shape << QPointF(-2, 0.2);
            shape << QPointF(-0.8, 0.2);
            shape << QPointF(-0.2, 0.8);
            shape << QPointF(-0.4, 1);
            shape << QPointF(0.2, 1);
            shape << QPointF(0.2, 0.4);
            shape << QPointF(0, 0.6);
            shape << QPointF(-0.8, -0.2);
            shape << QPointF(-2, -0.2);
            path.addPolygon(shape);
        }
        if ((markingType & Turn_Right) != 0)
        {
            QPolygonF shape;
            shape << QPointF(-2, 0.2);
            shape << QPointF(-0.8, 0.2);
            shape << QPointF(0, -0.6);
            shape << QPointF(0.2, -0.4);
            shape << QPointF(0.2, -1);
            shape << QPointF(-0.4, -1);
            shape << QPointF(-0.2, -0.8);
            shape << QPointF(-0.8, -0.2);
            shape << QPointF(-2, -0.2);
            path.addPolygon(shape);
        }
        if ((markingType & Turn_U) != 0 && 
            (markingType & Turn_No) == 0 && (markingType & Turn_Right) == 0)
        {
            path.moveTo(QPointF(-1.4, -0.2));
            path.arcTo(QRectF(-1.6, 0.2, 0.4, 0.4), 90, -180);
            path.arcTo(QRectF(-2, -0.2, 1.2, 1.2), -90, 180);
            path.closeSubpath();
            QPolygonF truck;
            truck << QPointF(-2, 0.2) << QPointF(-1.4, 0.2) << QPointF(-1.4, -0.2) << QPointF(-2, -0.2);
            path.addPolygon(truck);
            path.moveTo(-1.4, 0.4);
            path.lineTo(-1.4, 1.2);
            path.lineTo(-1.7, 0.8);
            path.closeSubpath();
        }
        setPath(path);
    }
}

