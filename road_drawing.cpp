#include "road_drawing.h"

#include <QGraphicsPathItem>
#include <qevent.h>

#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
#include "curve_fitting.h"

extern std::weak_ptr<RoadRunner::Road> g_PointerRoad;
extern double g_PointerRoadS;

RoadDrawingSession::RoadDrawingSession(QGraphicsView* aView) :
    view(aView), scene(aView->scene()), world(World::Instance())
{
    previewItem = scene->addPath(ctrlPath);
    hintItem = scene->addPath(hintPath);
    QPen hintPen;
    hintPen.setColor(QColor(0, 200, 100, 80));
    hintPen.setStyle(Qt::DotLine);
    hintItem->setPen(hintPen);
    cursorItem = scene->addEllipse(-2, -2, 4, 4);

    ctrlPoints.push_back(QPointF());
}

bool RoadDrawingSession::Update(QMouseEvent* event)
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
    float maxSnapDist = ctrlPoints.size() % 2 == 1 ? 5 : 1e9;
    SnapCtrlPoint(maxSnapDist);
    cursorItem->setPos(ctrlPoints.back());

    // TODO: incremental path?
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

void RoadDrawingSession::Complete()
{
    CreateRoad();
}

RoadDrawingSession::~RoadDrawingSession()
{
    scene->removeItem(previewItem);
    scene->removeItem(hintItem);
    scene->removeItem(cursorItem);
}

void RoadDrawingSession::SnapCtrlPoint(float maxOffset)
{
    QPointF& nextPoint = ctrlPoints.back();

    QVector2D nextDir;

    if (ctrlPoints.size() == 1)
    {
        startDir.reset();
        extendFromStart.reset();

        auto g_road = g_PointerRoad.lock();
        if (g_road != nullptr)
        {
            // Do point snap
            double snapS = g_PointerRoadS;
            if (g_PointerRoadS < SnapToEndThreshold)
            {
                snapS = 0;
                auto grad = g_road->generated.ref_line.get_grad_xy(snapS);
                startDir = std::make_unique<QVector2D>(-grad[0], -grad[1]);
                extendFromStart = g_PointerRoad;
                extendFromStartContact = odr::RoadLink::ContactPoint_Start;
            }
            else if (g_PointerRoadS > g_road->Length() - SnapToEndThreshold)
            {
                snapS = g_road->Length();
                auto grad = g_road->generated.ref_line.get_grad_xy(snapS);
                startDir = std::make_unique<QVector2D>(grad[0], grad[1]);
                extendFromStart = g_PointerRoad;
                extendFromStartContact = odr::RoadLink::ContactPoint_End;
            }

            auto snapped = g_road->generated.ref_line.get_xy(snapS);
            nextPoint.setX(snapped[0]);
            nextPoint.setY(snapped[1]);
        }

        return;
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
            if (g_PointerRoadS < SnapToEndThreshold)
            {
                snapS = 0;
                joinAtEnd = g_PointerRoad;
                joinAtEndContact = odr::RoadLink::ContactPoint_Start;
            }
            else if (g_PointerRoadS > g_road->Length() - SnapToEndThreshold)
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
}

void RoadDrawingSession::CreateRoad()
{
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

    RoadRunner::RoadProfile config(1, 0, 1, 0);
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

    bool standaloneRoad = true;

    if (!extendFromStart.expired())
    {
        standaloneRoad = false;
        auto toExtend = extendFromStart.lock();
        int joinResult = RoadRunner::Road::JoinRoads(toExtend, extendFromStartContact,
            newRoad, odr::RoadLink::ContactPoint_Start);
        if (joinResult != 0)
        {
            spdlog::warn("Extend error {}", joinResult);
        }

        toExtend->GenerateAllSectionGraphics();
        newRoad = toExtend;
    }

    if (!joinAtEnd.expired())
    {
        standaloneRoad = false;
        auto toJoin = joinAtEnd.lock();
        int joinResult = RoadRunner::Road::JoinRoads(toJoin, joinAtEndContact, 
            newRoad, odr::RoadLink::ContactPoint_End);
        if (joinResult != 0)
        {
            spdlog::warn("Join error {}", joinResult);
        }

        toJoin->GenerateAllSectionGraphics();
    }

    if (standaloneRoad)
    {
        newRoad->GenerateAllSectionGraphics();
        world->allRoads.insert(newRoad);
    }
}