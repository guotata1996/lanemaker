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


namespace RoadRunner
{
    TemporaryGraphics::TemporaryGraphics(const odr::Line3D& boundaryL, const odr::Line3D& boundaryR, QColor color)
    {
        graphicsIndex = g_mapViewGL->AddQuads(boundaryL, boundaryR, color);
    }

    TemporaryGraphics::TemporaryGraphics(const odr::Line3D& center, double width, QColor color)
    {
        odr::Line3D left(center.size());
        odr::Line3D right(center.size());
        for (int i = 0; i != center.size(); ++i)
        {
            auto prev = i == 0 ? center[i] : center[i - 1];
            auto next = i == center.size() - 1 ? center[i] : center[i + 1];
            auto tan = odr::sub(next, prev);
            odr::Vec2D radial{ -tan[1], tan[0] };
            radial = odr::normalize(radial);
            left[i] = odr::add(center[i], odr::Vec3D{ radial[0], radial[1], 0 });
            right[i] = odr::add(center[i], odr::Vec3D{ -radial[0], -radial[1], 0 });
        }
        graphicsIndex = g_mapViewGL->AddQuads(left, right, color);
    }

    TemporaryGraphics::~TemporaryGraphics()
    {
        g_mapViewGL->RemoveItem(graphicsIndex);
    }

    SectionGraphics::SectionGraphics(std::shared_ptr<RoadRunner::Road> road,
        const odr::LaneSection& laneSection,
        double sBegin, double sEnd)
    {        
        odr::Road& gen = road->generated;
        bool biDirRoad = gen.rr_profile.HasSide(-1) && gen.rr_profile.HasSide(1);
        sMin = std::min(sBegin, sEnd);
        sMax = std::max(sBegin, sEnd);
        for (const auto& id2Lane : laneSection.id_to_lane)
        {
            const auto& lane = id2Lane.second;
            if (lane.type == "median" || lane.type == "driving")
            {
                odr::Line3D innerBorder, outerBorder;
                gen.get_lane_border_line(lane, sMin, sMax, 0.1f, outerBorder, innerBorder);

                int nSubDivisions = std::min(10, std::max(1, static_cast<int>(outerBorder.size()) / 3));
                for (int d = 0; d != nSubDivisions; ++d)
                {
                    double segMin = (sMin * (nSubDivisions - d) + sMax * d) / nSubDivisions;
                    double segMax = (sMin * (nSubDivisions - 1 - d) + sMax * (d + 1)) / nSubDivisions;
                    allSpatialIndice.push_back(SpatialIndexer::Instance()->Index(gen, lane, segMin, segMax));
                }
                
                auto surfaceIndex = g_mapViewGL->AddQuads(innerBorder, outerBorder, 
                    lane.type == "median" ? Qt::yellow : Qt::darkGray);
                allGraphicsIndice.push_back(surfaceIndex);
                if (lane.type != "median")
                {
                    allHighlightGraphicsIndice.push_back(surfaceIndex);
                }
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

                std::vector<odr::Line3D> leftLines, rightLines;
                std::vector<std::string> colors;

                if (groupIt->type == "solid" || groupIt->type == "curb")
                {
                    auto markingLine = gen.get_lane_marking_line(lane, groupsBegin, groupsEnd, groupIt->width, 0.1f);
                    leftLines.push_back(markingLine.first);
                    rightLines.push_back(markingLine.second);
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
                            auto markingLine = gen.get_lane_marking_line(lane,
                                sBeginInSegment, sEndInSegment, groupIt->width, 0.1f);
                            leftLines.push_back(markingLine.first);
                            rightLines.push_back(markingLine.second);
                            colors.push_back(groupIt->color);
                        }
                    }
                }
                else
                {
                    continue;
                }

                for (int i = 0; i != colors.size(); ++i)
                {
                    Qt::GlobalColor color = colors[i] == "yellow" ? Qt::yellow :
                        (colors[i] == "white" ? Qt::white : Qt::lightGray);
                    allGraphicsIndice.push_back(g_mapViewGL->AddQuads(leftLines[i], rightLines[i], color));
                }
            }
        }

        // Draw road objects
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

                        uint8_t side = id_object.first == std::to_string(odr::RoadLink::ContactPoint_Start) ? 1 : -1;
                        auto l1 = gen.get_side_border_line(side, s1, s2, false, 0.1);
                        auto l2 = gen.get_side_border_line(side, s1, s2, true, 0.1);
                        for (auto& p : l1)
                        {
                            p[2] += 0.01;
                        }
                        for (auto& p : l2)
                        {
                            p[2] += 0.01;
                        }
                        allGraphicsIndice.push_back(g_mapViewGL->AddQuads(l1, l2, Qt::white));
                    }
                    else if (id_object.second.subtype == "arrow")
                    {
                        auto sMid = (sMin + sMax) / 2;  // arrow always placed at mid at current section
                        auto pt = gen.get_surface_pt(sMid, id_object.second.t0);
                        auto hdg = gen.ref_line.get_hdg(sMid);
                        int arrowType = std::stoi(id_object.second.name);
                        QTransform arrowTransform;
                        arrowTransform.translate(pt[0], pt[1]);
                        arrowTransform.rotate(180 / M_PI * id_object.second.hdg);

                        for (auto poly : ArrowShape(arrowType))
                        {
                            auto transformed = arrowTransform.map(poly);
                            odr::Line3D shape3;
                            shape3.resize(transformed.size());
                            for (int i = 0; i != transformed.size(); ++i)
                            {
                                shape3[i] = odr::Vec3D{ transformed[i].x(), transformed[i].y(), pt[2] + 0.01};
                            }
                            allGraphicsIndice.push_back(g_mapViewGL->AddPoly(shape3, Qt::white));
                        }
                    }
                    else
                    {
                        spdlog::warn("roadMark subtype {} isn't supported!", id_object.second.subtype);
                        continue;
                    }
                }
            }
        }

        //refLineHint = new QGraphicsPathItem(this);
        //refLineHint->setPen(QPen(Qt::green, 0.3, Qt::SolidLine));
        //refLineHint->hide();
        //auto lineAppox = gen.ref_line.get_line(std::min(sBegin, sEnd), std::max(sBegin, sEnd), 0.1f);
        //refLinePath = CreateRefLinePath(lineAppox);
        //std::reverse(lineAppox.begin(), lineAppox.end());
        //refLinePathReversed = CreateRefLinePath(lineAppox);
        //refLineHint->setPath(refLinePath);
        // TODO
    }

    SectionGraphics::~SectionGraphics()
    {
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

    void SectionGraphics::EnableHighlight(bool enabled)
    {
        QColor newColor = enabled ? Qt::lightGray : Qt::darkGray;
        for (auto id : allHighlightGraphicsIndice)
        {
            g_mapViewGL->UpdateItem(id, newColor);
        }
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

    void SectionGraphics::updateIndexingInfo(std::string newRoadID, int mult, double shift)
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
                face.sBegin = face.sBegin * mult + shift;
                face.sEnd = face.sEnd * mult + shift;
            }
        }
        //refLineHint->setPath(sBegin < sEnd ? refLinePath : refLinePathReversed);
        sMin = sMin * mult + shift;
        sMax = sMax * mult + shift;
        if (mult < 0)
        {
            std::swap(sMin, sMax);
        }
        assert(sMin < sMax);
    }

    double SectionGraphics::Length() const
    {
        assert(sMin < sMax);
        return sMax - sMin;
    }
    
    JunctionGraphics::JunctionGraphics(const odr::Line2D& boundary, double elevation)
    {
        odr::Line3D boundary3;
        boundary3.resize(boundary.size());
        for (int i = 0; i != boundary.size(); ++i)
        {
            boundary3[i] = odr::Vec3D{ boundary[i][0], boundary[i][1], elevation };
        }
        allGraphicsIndice.push_back(g_mapViewGL->AddPoly(boundary3, Qt::darkGray));
    }

    JunctionGraphics::JunctionGraphics(const std::vector<std::pair<odr::Line2D, odr::Line2D>>& boundary, double elevation)
    {
        QPainterPath path;

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

            allGraphicsIndice.push_back(g_mapViewGL->AddQuads(TwoDTo3D(dualSides.first, elevation), 
                TwoDTo3D(dualSides.second, elevation), Qt::darkGray));

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

                odr::Line3D boundary3d;
                boundary3d.push_back(odr::Vec3D{ p1[0], p1[1], elevation + 0.01 });
                boundary3d.push_back(odr::Vec3D{ pM1[0], pM1[1], elevation + 0.01 });
                boundary3d.push_back(odr::Vec3D{ p2[0], p2[1], elevation + 0.01 });
                boundary3d.push_back(odr::Vec3D{ p4[0], p4[1], elevation + 0.01 });
                boundary3d.push_back(odr::Vec3D{ pM2[0], pM2[1], elevation + 0.01 });
                boundary3d.push_back(odr::Vec3D{ p3[0], p3[1], elevation + 0.01 });
                allGraphicsIndice.push_back(g_mapViewGL->AddPoly(boundary3d, Qt::lightGray));
            }
        }
    }

    JunctionGraphics::~JunctionGraphics()
    {
        for (auto index : allGraphicsIndice)
        {
            g_mapViewGL->RemoveItem(index);
        }
    }

    //void JunctionGraphics::hoverEnterEvent(QGraphicsSceneHoverEvent* evt)
    //{
    //    setBrush(Qt::NoBrush);
    //    junctionElevation = zValue();
    //    setZValue(128.1); // higher than highlighted linked road to properly receive hoverLeaveEvent
    //}

    //void JunctionGraphics::hoverLeaveEvent(QGraphicsSceneHoverEvent* evt)
    //{
    //    setBrush(DefaultBrush);
    //    setZValue(junctionElevation);
    //}

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

    namespace
    {
        std::vector<QPolygonF> ArrowShape(int markingType)
        {
            std::vector<QPolygonF> rtn;
            if ((markingType & DeadEnd) != 0)
            {
                QPolygonF stem, cross1, cross2;
                stem << QPointF(-2, 0.2) << QPointF(1, 0.2) << QPointF(1, -0.2) << QPointF(-2, -0.2);
                rtn.push_back(stem);
                cross1 << QPointF(0.4, 1) << QPointF(0.6, 1) << QPointF(1.6, -1) << QPointF(1.4, -1);
                rtn.push_back(cross1);
                cross2 << QPointF(1.4, 1) << QPointF(1.6, 1) << QPointF(0.6, -1) << QPointF(0.4, -1);
                rtn.push_back(cross2);
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
                rtn.push_back(shape);
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
                rtn.push_back(shape);
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
                rtn.push_back(shape);
            }

            if ((markingType & Turn_U) != 0 &&
                (markingType & Turn_Right) == 0)
            {
                QPainterPath path;
                path.moveTo(QPointF(-1.4, -0.2));
                path.arcTo(QRectF(-1.6, 0.2, 0.4, 0.4), 90, -180);
                path.arcTo(QRectF(-2, -0.2, 1.2, 1.2), -90, 180);
                path.closeSubpath();

                QPolygonF shape;
                for (double pct = 0; pct < 1; pct += 0.01)
                {
                    shape.push_back(path.pointAtPercent(pct));
                }
                rtn.push_back(shape);

                QPolygonF trunk;
                trunk << QPointF(-2, 0.2) << QPointF(-1.4, 0.2) << QPointF(-1.4, -0.2) << QPointF(-2, -0.2);
                rtn.push_back(trunk);

                QPolygonF arrow;
                arrow << QPointF(-1.4, 0.4) << QPointF(-1.4, 1.2) << QPointF(-1.7, 0.8);
                rtn.push_back(arrow);
            }
            return rtn;
        }
    }
}

