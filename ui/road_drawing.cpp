#include "road_drawing.h"

#include <QGraphicsPathItem>
#include <qevent.h>

#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
#include "curve_fitting.h"
#include "junction.h"

#include "stats.h"

extern std::weak_ptr<RoadRunner::Road> g_PointerRoad;
extern double g_PointerRoadS;

extern RoadRunner::SectionProfile activeLeftSetting;
extern RoadRunner::SectionProfile activeRightSetting;

extern double g_zoom;

float RoadDrawingSession::SnapDistFromScale() const
{
    return CustomCursorItem::SnapRadiusPx / g_zoom;
}

double CustomCursorItem::SnapRadiusPx = 20;
double CustomCursorItem::InitialRadius = 2;

void CustomCursorItem::EnableHighlight(bool enable)
{
    setBrush(enable ? QBrush(Qt::red, Qt::SolidPattern) : Qt::NoBrush);
}

void CustomCursorItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    this->setScale(SnapRadiusPx / InitialRadius / g_zoom);
    QGraphicsEllipseItem::paint(painter, option, widget);
}

RoadCreationSession::RoadCreationSession(QGraphicsView* aView) :
    RoadDrawingSession(aView)
{
    previewItem = scene->addPath(ctrlPath);
    hintItem = scene->addPath(hintPath);
    QPen hintPen;
    hintPen.setColor(QColor(0, 200, 100, 80));
    hintPen.setStyle(Qt::DotLine);
    hintItem->setPen(hintPen);
    cursorItem = new CustomCursorItem;
    scene->addItem(cursorItem);

    ctrlPoints.push_back(QPointF());
}

bool RoadCreationSession::Update(QMouseEvent* event)
{
    QPointF scenePos = view->mapToScene(event->pos().x(), event->pos().y());

    if (event->button() == Qt::MouseButton::LeftButton)
    {
        ctrlPoints.push_back(scenePos);
        if (ctrlPoints.size() >= 3 && !joinAtEnd.expired())
        {
            return false;
        }
    }
    else if (event->button() == Qt::MouseButton::RightButton)
    {
        if (ctrlPoints.size() > 1)
        {
            // At least keep one
            ctrlPoints.pop_back();
        }
    }
    else if (event->button() == Qt::MouseButton::NoButton)
    {
        ctrlPoints.back().setX(scenePos.x());
        ctrlPoints.back().setY(scenePos.y());
    }

    hintPath.clear();
    float maxSnapDist = ctrlPoints.size() % 2 == 1 ? SnapDistFromScale() : 1e9;
    bool onExisting = SnapCtrlPoint(maxSnapDist);
    cursorItem->setPos(ctrlPoints.back());
    cursorItem->EnableHighlight(onExisting);

    ctrlPath.clear();
    if (!ctrlPoints.empty())
    {
        ctrlPath.moveTo(ctrlPoints[0]);
    }
    int i = 1;
    for (; i + 2 < ctrlPoints.size(); i += 2)
    {
        ctrlPath.quadTo(ctrlPoints[i], ctrlPoints[i + 1]);
    }
    if (i == ctrlPoints.size() - 1)
    {
        ctrlPath.lineTo(ctrlPoints[i]);
    }
    else if (i == ctrlPoints.size() - 2)
    {
        ctrlPath.quadTo(ctrlPoints[i], ctrlPoints[i + 1]);
    }

    previewItem->setPath(ctrlPath);
    hintItem->setPath(hintPath);

    return true;
}

void RoadCreationSession::Complete()
{
    Stats s("LaneSegmentGraphics Created");
    CreateRoad();
}

RoadCreationSession::~RoadCreationSession()
{
    scene->removeItem(previewItem);
    scene->removeItem(hintItem);
    scene->removeItem(cursorItem);
}

bool RoadCreationSession::SnapCtrlPoint(float maxOffset)
{
    QPointF& nextPoint = ctrlPoints.back();

    QVector2D nextDir;
    bool onExisting = false;
    const double snapThreshold = SnapDistFromScale();

    if (ctrlPoints.size() == 1)
    {
        startDir.reset();
        extendFromStart.reset();
        extendFromStartContact = odr::RoadLink::ContactPoint_None;

        auto g_road = g_PointerRoad.lock();
        if (g_road != nullptr)
        {
            // Do point snap
            double snapS = g_PointerRoadS;
            if (g_PointerRoadS < snapThreshold)
            {
                snapS = 0;
                auto grad = g_road->generated.ref_line.get_grad_xy(snapS);
                startDir = std::make_unique<QVector2D>(-grad[0], -grad[1]);
                extendFromStart = g_PointerRoad;
                extendFromStartContact = odr::RoadLink::ContactPoint_Start;
            }
            else if (g_PointerRoadS > g_road->Length() - snapThreshold)
            {
                snapS = g_road->Length();
                auto grad = g_road->generated.ref_line.get_grad_xy(snapS);
                startDir = std::make_unique<QVector2D>(grad[0], grad[1]);
                extendFromStart = g_PointerRoad;
                extendFromStartContact = odr::RoadLink::ContactPoint_End;
            }

            if (extendFromStartContact != odr::RoadLink::ContactPoint_None)
            {
                // only snap to ends
                auto snapped = g_road->generated.ref_line.get_xy(snapS);
                nextPoint.setX(snapped[0]);
                nextPoint.setY(snapped[1]);
                onExisting = true;
            }
        }

        return onExisting;
    }
    else if (ctrlPoints.size() == 2)
    {
        // Extend from existing
        if (startDir != nullptr)
        {
            nextDir = *startDir;
        }
    }
    else
    {
        joinAtEnd.reset();
        auto g_road = g_PointerRoad.lock();

        if (g_road != nullptr)
        {
            // Join to existing
            double snapS;
            if (g_PointerRoadS < snapThreshold)
            {
                snapS = 0;
                joinAtEnd = g_PointerRoad;
                joinAtEndContact = odr::RoadLink::ContactPoint_Start;
            }
            else if (g_PointerRoadS > g_road->Length() - snapThreshold)
            {
                snapS = g_road->Length();
                joinAtEnd = g_PointerRoad;
                joinAtEndContact = odr::RoadLink::ContactPoint_End;
            }
            if (!joinAtEnd.expired())
            {
                auto snapped = g_road->generated.ref_line.get_xy(snapS);
                nextPoint.setX(snapped[0]);
                nextPoint.setY(snapped[1]);
                onExisting = true;
            }
        }
        else
        {
            nextDir = QVector2D(ctrlPoints[ctrlPoints.size() - 2] - ctrlPoints[ctrlPoints.size() - 3]);
        }
    }

    if (nextDir.lengthSquared() > 0.01f)
    {
        nextDir.normalize();
        hintPath.moveTo(ctrlPoints[ctrlPoints.size() - 2]);
        hintPath.lineTo(ctrlPoints[ctrlPoints.size() - 2] + QPointF(nextDir.x(), nextDir.y()) * 100);

        // Do projection
        QPointF lastPoint = ctrlPoints[ctrlPoints.size() - 2];
        QVector2D last2Next(nextPoint - lastPoint);
        auto projLength = QVector2D::dotProduct(last2Next, nextDir);
        QVector2D projected = QVector2D(lastPoint) + projLength * nextDir;
        if (projected.distanceToPoint(QVector2D(nextPoint)) < maxOffset)
        {
            nextPoint.setX(projected.x());
            nextPoint.setY(projected.y());
        }
    }

    return onExisting;
}

void RoadCreationSession::CreateRoad()
{
    if (!extendFromStart.expired() && extendFromStart.lock() == joinAtEnd.lock())
    {
        spdlog::warn("Self-loop is not supported!");
        return;
    }

    odr::RefLine refLine("", 0);
    double cumLength = 0;
    /*Discard last CtrlPoints since we are gonna auto connect*/
    int validCtrlPoints = joinAtEnd.expired() ? ctrlPoints.size() - 1 : ctrlPoints.size() - 2;
    for (int i = 0; i + 2 < validCtrlPoints; i += 2)
    {
        std::unique_ptr<odr::RoadGeometry> localGeometry;

        QVector2D start(ctrlPoints[i]);
        QVector2D middle(ctrlPoints[i + 1]);
        QVector2D end(ctrlPoints[i + 2]);
        odr::Vec2D point1{ start.x(), start.y() };
        odr::Vec2D point2{ middle.x(), middle.y() };
        odr::Vec2D point3{ end.x(), end.y() };

        if (middle.distanceToLine(start, end - start) < 0.1f)
        {
            localGeometry = std::make_unique<odr::Line>(cumLength, point1, point3);
        }
        else
        {
            localGeometry = std::make_unique<odr::ParamPoly3>(cumLength, point1, point2, point2, point3);
        }
        double cumLengthPrev = cumLength;
        cumLength += localGeometry->length;
        refLine.s0_to_geometry.emplace(cumLengthPrev, std::move(localGeometry));
    }
    refLine.length = cumLength;

    RoadRunner::RoadProfile config(
        activeLeftSetting.laneCount, activeLeftSetting.offsetx2, 
        activeRightSetting.laneCount, activeRightSetting.offsetx2);
    std::shared_ptr<RoadRunner::Road> newRoad;
    if (cumLength == 0)
    {
        auto p0 = ctrlPoints[0];
        auto p1 = ctrlPoints[1];
        auto p2 = ctrlPoints[2];
        odr::Vec2D pos0{ p0.x(), p0.y() };
        odr::Vec2D pos1{ p1.x(), p1.y() };
        odr::Vec2D dir0 = odr::normalize(odr::sub(pos1, pos0));
        odr::Vec2D pos2{ p2.x(), p2.y() };
        auto toJoin = joinAtEnd.lock();
        odr::Vec2D dir2 = toJoin->generated.ref_line.get_grad_xy(
            joinAtEndContact == odr::RoadLink::ContactPoint_Start ? 0 : toJoin->Length());
        if (joinAtEndContact == odr::RoadLink::ContactPoint_End)
        {
            dir2 = odr::negate(dir2);
        }
        odr::normalize(dir2);
        auto roadGeometry = ConnectLines(pos0, dir0, pos2, dir2);
        // TODO: if roadGeometry too long, return failure
        newRoad = std::make_shared<RoadRunner::Road>(config, roadGeometry);
    }
    else
    {
        newRoad = std::make_shared<RoadRunner::Road>(config, refLine);
    }
    newRoad->GenerateAllSectionGraphics();

    bool standaloneRoad = true;
    bool joinPlusExtend = false;
    // Which part of newRoad will be newly-created?
    double newPartBegin = 0;
    if (!extendFromStart.expired())
    {
        standaloneRoad = false;
        joinPlusExtend = true;
        auto toExtend = extendFromStart.lock();
        newPartBegin = toExtend->Length();
        int joinResult = RoadRunner::Road::JoinRoads(toExtend, extendFromStartContact,
            newRoad, odr::RoadLink::ContactPoint_Start);
        if (joinResult != 0)
        {
            spdlog::warn("Extend error {}", joinResult);
        }

        newRoad = toExtend;
    }

    if (!joinAtEnd.expired())
    {
        if (joinPlusExtend)
        {
            world->allRoads.erase(newRoad);
        }

        standaloneRoad = false;
        auto toJoin = joinAtEnd.lock();
        newPartBegin = toJoin->Length();

        int joinResult = RoadRunner::Road::JoinRoads(toJoin, joinAtEndContact, 
            newRoad, odr::RoadLink::ContactPoint_End);
        if (joinResult != 0)
        {
            spdlog::warn("Join error {}", joinResult);
        }
        newRoad = toJoin;
    }

    if (standaloneRoad)
    {
        world->allRoads.insert(newRoad);
    }

    // Check if we need a junction
    const double JunctionExtaTrim = 10;
    while (true)
    {
        auto overlap = newRoad->FirstOverlapNonJunction(newPartBegin, newRoad->Length());
        if (overlap == nullptr)
        {
            break;
        }

        auto road2 = overlap->road2.lock();

        bool canCreateJunction = true; // If not, this collision will end up as an overlap
        double sBegin1 = overlap->sBegin1 - JunctionExtaTrim;
        double sEnd1 = overlap->sEnd1 + JunctionExtaTrim;
        double sBegin2 = overlap->sBegin2 - JunctionExtaTrim;
        double sEnd2 = overlap->sEnd2 + JunctionExtaTrim;

        if (sBegin1 < 0)
        {
            if (newRoad->predecessorJunction != nullptr)
            {
                // Don't make junction if one is already too close
                canCreateJunction = false;
            }
            else
            {
                sBegin1 = 0; // T junction
            }
        }
        if (sEnd1 > newRoad->Length())
        {
            if (newRoad->successorJunction != nullptr)
            {
                // Don't make junction if one is already too close
                canCreateJunction = false;
            }
            else
            {
                sEnd1 = newRoad->Length();  // T junction
            }
        }
        if (sBegin2 < 0)
        {
            if (road2->predecessorJunction != nullptr)
            {
                // Don't make junction if one is already too close
                canCreateJunction = false;
            }
            else
            {
                sBegin2 = 0;
            }
        }
        if (sEnd2 > road2->Length())
        {
            if (road2->successorJunction != nullptr)
            {
                // Don't make junction if one is already too close
                canCreateJunction = false;
            }
            else
            {
                sEnd2 = road2->Length();
            }
        }

        std::shared_ptr<RoadRunner::Road> newRoadBeforeJunction, newRoadPastJunction;
        std::shared_ptr<RoadRunner::Road> road2BeforeJunction, road2PastJunction;

        if (canCreateJunction)
        {
            if (sEnd1 != newRoad->Length())
            {
                newRoadPastJunction = RoadRunner::Road::SplitRoad(newRoad, sEnd1);
                world->allRoads.insert(newRoadPastJunction);
            }
            if (sBegin1 != 0)
            {
                RoadRunner::Road::SplitRoad(newRoad, sBegin1);
                newRoadBeforeJunction = newRoad;
            }
            else
            {
                world->allRoads.erase(newRoad);
            }

            if (newRoadBeforeJunction == nullptr && newRoadPastJunction == nullptr)
            {
                // newRoad is too short
                canCreateJunction = false;
            }

            if (sEnd2 != road2->Length())
            {
                road2PastJunction = RoadRunner::Road::SplitRoad(road2, sEnd2);
                world->allRoads.insert(road2PastJunction);
            }
            if (sBegin2 != 0)
            {
                RoadRunner::Road::SplitRoad(road2, sBegin2);
                road2BeforeJunction = road2;
            }
            else
            {
                world->allRoads.erase(road2);
            }
            if (road2BeforeJunction == nullptr && road2PastJunction == nullptr)
            {
                // road2 is too short
                canCreateJunction = false;
            }
        }

        std::vector<RoadRunner::ConnectionInfo> junctionInfo;
        if (canCreateJunction)
        {
            if (newRoadBeforeJunction != nullptr)
            {
                junctionInfo.push_back(RoadRunner::ConnectionInfo{ newRoadBeforeJunction, odr::RoadLink::ContactPoint_End });
            }
            if (newRoadPastJunction != nullptr)
            {
                junctionInfo.push_back(RoadRunner::ConnectionInfo{ newRoadPastJunction, odr::RoadLink::ContactPoint_Start });
            }
            if (road2BeforeJunction != nullptr)
            {
                junctionInfo.push_back(RoadRunner::ConnectionInfo{ road2BeforeJunction, odr::RoadLink::ContactPoint_End });
            }
            if (road2PastJunction != nullptr)
            {
                junctionInfo.push_back(RoadRunner::ConnectionInfo{ road2PastJunction, odr::RoadLink::ContactPoint_Start });
            }
            if (junctionInfo.size() < 3)
            {
                // 2-road junction, should really be a Join
                canCreateJunction = false;
            }
        }

        if (canCreateJunction)
        {
            auto junction = std::make_shared<RoadRunner::Junction>();
            junction->CreateFrom(junctionInfo);
        }
        else
        {
            spdlog::info("Collision detected, but cannot create Junction");
        }

        if (newRoadPastJunction == nullptr)
        {
            break;
        }
        newRoad = newRoadPastJunction;
        newPartBegin = 0;
    }
}