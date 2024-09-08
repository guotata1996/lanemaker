#include "road_drawing.h"
#include "stats.h"
#include "junction.h"

#include <QGraphicsPathItem>
#include <qevent.h>


extern std::weak_ptr<RoadRunner::Road> g_PointerRoad;
extern double g_PointerRoadS;

RoadDestroySession::RoadDestroySession(QGraphicsView* aView):
    RoadDrawingSession(aView)
{
    hintItemLeft = scene->addPolygon(hintPolygonLeft);
    hintItemLeft->setZValue(129);
    QPen pen;
    pen.setColor(Qt::red);
    pen.setWidthF(0.5);
    hintItemLeft->setPen(pen);

    hintItemRight = scene->addPolygon(hintPolygonRight);
    hintItemRight->setZValue(129);
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

bool RoadDestroySession::Update(const RoadRunner::MouseAction& evt)
{
    QPointF scenePos(evt.sceneX, evt.sceneY);
    auto g_road = g_PointerRoad.lock();
    if (g_road != nullptr)
    {
        bool onSegBoundary;
        auto snapped = g_road->generated.ref_line.get_xy(GetAdjustedS(&onSegBoundary));
        cursorItem->setPos(snapped[0], snapped[1]);
        cursorItem->EnableHighlight(onSegBoundary ? 
            RoadDrawingSession::Snap_Point : RoadDrawingSession::Snap_Line);
    }
    else
    {
        cursorItem->setPos(scenePos);
        cursorItem->EnableHighlight(RoadDrawingSession::Snap_Nothing);
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
            auto borders = target->generated.get_both_dirs_poly(fromS, toS, 1);
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
    if (evt.button == Qt::LeftButton 
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
            if (evt.type == QEvent::Type::MouseButtonPress)
            {
                s2 = std::make_unique<double>(GetAdjustedS());
            }
            else if (evt.type == QEvent::Type::MouseButtonDblClick)
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

bool RoadDestroySession::Complete()
{
    if (s2 == nullptr)
    {
        return true;
    }
    Stats s("LaneGraphics Created");

    double from = std::min(*s1, *s2);
    double to = std::max(*s1, *s2);
    auto target = targetRoad.lock();
    auto predJunction = target->predecessorJunction;
    auto succJunction = target->successorJunction;

    if (from == 0 && to == target->Length())
    {
        auto nRemoved = world->allRoads.erase(target);
        if (nRemoved != 1) throw std::logic_error("Road to destroy is not found in world!");
    }
    else if (from == 0)
    {
        auto secondHalf = RoadRunner::Road::SplitRoad(target, to);
        auto nRemoved = world->allRoads.erase(target);
        if (nRemoved != 1) throw std::logic_error("Road to destroy is not found in world!");
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

    // Check if removal negatively affects any related junction
    target.reset();
    if (predJunction != nullptr &&
        predJunction->generationError != RoadRunner::Junction_NoError
        ||
        succJunction != nullptr &&
        succJunction->generationError != RoadRunner::Junction_NoError)
    {
        spdlog::warn("Abort: Removal of this road causes a junction to be invalid!");
        return false;
    }

    if (predJunction != nullptr && !predJunction.unique())
    {
        predJunction->CheckForDegeneration();
    }
    if (succJunction != nullptr && !succJunction.unique())
    {
        succJunction->CheckForDegeneration();
    }

    return true;
}