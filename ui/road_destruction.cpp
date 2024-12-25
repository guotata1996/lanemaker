#include "road_drawing.h"
#include "stats.h"
#include "junction.h"
#include "map_view_gl.h"

#include <QGraphicsPathItem>
#include <qevent.h>


bool RoadDestroySession::Update(const RoadRunner::MouseAction& evt)
{
    auto g_road = GetPointerRoad();
    if (g_road != nullptr)
    {
        bool onSegBoundary;
        auto snapped = g_road->generated.get_xyz(GetAdjustedS(&onSegBoundary), 0, 0);
        cursorItem->SetTranslation(snapped);
        cursorItem->EnableHighlight(onSegBoundary ? 
            RoadDrawingSession::Snap_Point : RoadDrawingSession::Snap_Line);
    }
    else
    {
        auto groundPos = RoadRunner::g_PointerOnGround;
        cursorItem->SetTranslation({groundPos[0], groundPos[1], 0});
        cursorItem->EnableHighlight(RoadDrawingSession::Snap_Nothing);
    }
    SetHighlightTo(g_road);

    auto target = targetRoad.lock();
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
                if (s2 == nullptr || std::abs(*s1 - *s2) < 1.0)
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

    UpdateHint();

    return true;
}

bool RoadDestroySession::Update(const RoadRunner::KeyPressAction& evt)
{
    if (evt.key == Qt::Key_Escape)
    {
        if (s2 != nullptr)
        {
            s2.reset();
            UpdateHint();
        }
        else if (s1 != nullptr)
        {
            s1.reset();
            targetRoad.reset();
            UpdateHint();
        }
        else
        {
            return false;
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

    if (predJunction != nullptr && !predJunction.unique() && predJunction->CanDegerate())
    {
        predJunction->Degenerate();
    }
    if (succJunction != nullptr && !succJunction.unique() && succJunction->CanDegerate())
    {
        succJunction->Degenerate();
    }

    UpdateEndMarkings();
    return true;
}

void RoadDestroySession::UpdateHint()
{
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
        else if (target == GetPointerRoad())
        {
            double proposedS2 = GetAdjustedS();
            fromS = std::min(*s1, proposedS2);
            toS = std::max(*s1, proposedS2);
        }

        if (fromS >= 0)
        {
            // Can preview
            auto borders = target->generated.get_both_dirs_poly(fromS, toS, 0.25);
            for (auto& lBorder : borders.first)
            {
                lBorder[2] += 0.02;
            }
            for (auto& rBorder : borders.second)
            {
                rBorder[2] += 0.02;
            }
            hintPolygonLeft = borders.first;
            hintPolygonRight = borders.second;
        }
    }
    hintItemLeft.emplace(hintPolygonLeft, 0.2, Qt::red);
    hintItemRight.emplace(hintPolygonRight, 0.2, Qt::green);
}