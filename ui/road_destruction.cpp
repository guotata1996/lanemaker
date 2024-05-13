#include "road_drawing.h"

#include <QGraphicsPathItem>
#include <qevent.h>

#include "stats.h"

extern std::weak_ptr<RoadRunner::Road> g_PointerRoad;
extern double g_PointerRoadS;

RoadDestroySession::RoadDestroySession(QGraphicsView* aView):
    RoadDrawingSession(aView)
{
    hintItemLeft = scene->addPolygon(hintPolygonLeft);
    hintItemLeft->setZValue(10);
    QPen pen;
    pen.setColor(Qt::red);
    pen.setWidthF(0.5);
    hintItemLeft->setPen(pen);

    hintItemRight = scene->addPolygon(hintPolygonRight);
    hintItemRight->setZValue(10);
    pen.setColor(Qt::green);
    hintItemRight->setPen(pen);
}

RoadDestroySession::~RoadDestroySession()
{
    scene->removeItem(cursorItem);
    scene->removeItem(hintItemLeft);
    scene->removeItem(hintItemRight);
    SetHighlightTo(nullptr);
}

bool RoadDestroySession::Update(QMouseEvent* evt)
{
    QPointF scenePos = view->mapToScene(evt->pos().x(), evt->pos().y());
    auto g_road = g_PointerRoad.lock();
    if (g_road != nullptr)
    {
        auto snapped = g_road->generated.ref_line.get_xy(GetAdjustedS());
        cursorItem->setPos(snapped[0], snapped[1]);
        cursorItem->EnableHighlight(true);
    }
    else
    {
        cursorItem->setPos(scenePos);
        cursorItem->EnableHighlight(false);
    }
    cursorItem->show();
    SetHighlightTo(g_road);

    // Preview
    hintPolygonLeft.clear();
    hintPolygonRight.clear();
    auto target = targetRoad.lock();
    if (target != nullptr)
    {
        double fromS = -1;
        double toS;

        if (s2 != nullptr)
        {
            fromS = std::min(*s1, *s2);
            toS = std::max(*s1, *s2);
        }
        else if (target == g_road)
        {
            double proposedS2 = GetAdjustedS();
            fromS = std::min(*s1, proposedS2);
            toS = std::max(*s1, proposedS2);
        }

        if (fromS >= 0)
        {
            // Can preview
            auto borders = target->generated.get_road_border_line(fromS, toS, 1);
            auto& leftBorder = borders.first;
            auto& rightBorder = borders.second;
            for (const odr::Vec3D& p : leftBorder)
            {
                hintPolygonLeft.append(QPointF(p[0], p[1]));
            }
            for (const odr::Vec3D& p : rightBorder)
            {
                hintPolygonRight.append(QPointF(p[0], p[1]));
            }
        }
    }
    hintItemLeft->setPolygon(hintPolygonLeft);
    hintItemRight->setPolygon(hintPolygonRight);

    // Change target, s1, s2
    if (evt->button() == Qt::LeftButton 
        && g_road != nullptr && g_road->generated.junction == "-1")
    {
        if (g_road != target)
        {
            targetRoad = g_road;
            s1 = std::make_unique<double>(GetAdjustedS());
            s2.reset();
        }
        else
        {
            if (evt->type() == QEvent::Type::MouseButtonPress)
            {
                s2 = std::make_unique<double>(GetAdjustedS());
            }
            else if (evt->type() == QEvent::Type::MouseButtonDblClick)
            {
                if (s2 == nullptr)
                {
                    s1 = std::make_unique<double>(0);
                    s2 = std::make_unique<double>(target->Length());
                }
                else
                {
                    s2 = std::make_unique<double>(*s2 > *s1 ? target->Length() : 0);
                }
                
            }
        }
    }

    return true;
}

double RoadDestroySession::GetAdjustedS()
{
    auto g_road = g_PointerRoad.lock();
    const double snapThreshold = SnapDistFromScale();
    if (g_PointerRoadS < snapThreshold)
    {
        return 0;
    }
    else if (g_PointerRoadS > g_road->Length() - snapThreshold)
    {
        return g_road->Length();
    }
    return g_road->SnapToSegmentBoundary(g_PointerRoadS, snapThreshold);

}

void RoadDestroySession::Complete()
{
    if (s2 == nullptr)
    {
        return;
    }
    Stats s("LaneSegmentGraphics Created");

    double from = std::min(*s1, *s2);
    double to = std::max(*s1, *s2);
    auto target = targetRoad.lock();
    if (from == 0 && to == target->Length())
    {
        auto nRemoved = world->allRoads.erase(target);
        assert(nRemoved == 1);
    }
    else if (from == 0)
    {
        auto secondHalf = RoadRunner::Road::SplitRoad(target, to);
        auto nRemoved = world->allRoads.erase(target);
        assert(nRemoved == 1);
        world->allRoads.insert(secondHalf);
    }
    else if (to == target->Length())
    {
        RoadRunner::Road::SplitRoad(target, from);
    }
    else
    {
        auto third = RoadRunner::Road::SplitRoad(target, to);
        RoadRunner::Road::SplitRoad(target, from);

        world->allRoads.insert(third);
    }
}