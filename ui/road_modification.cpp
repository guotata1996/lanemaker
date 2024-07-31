#include "road_drawing.h"
#include "junction.h"

#include <qevent.h>
#include "CreateRoadOptionWidget.h"

extern std::weak_ptr<RoadRunner::Road> g_PointerRoad;
extern double g_PointerRoadS;

extern SectionProfileConfigWidget* g_createRoadOption;

RoadModificationSession::RoadModificationSession(QGraphicsView* aView) : RoadDestroySession(aView)
{}

bool RoadModificationSession::Update(const RoadRunner::MouseAction& evt)
{
    RoadDestroySession::Update(evt);

    if (!g_PointerRoad.expired() && evt.type == QEvent::Type::MouseButtonPress)
    {
        BeginPickingProfile();
    }
    else if (evt.type == QEvent::Type::MouseButtonRelease)
    {
        EndPickingProfile();
    }
    if (PickProfileMode())
    {
        ContinuePickingProfile();
    }
    return true;
}

bool RoadModificationSession::Complete()
{
    if (targetRoad.expired() || s2 == nullptr) return true;

    auto target = targetRoad.lock();
    if (std::min(*s1, *s2) == 0 &&
        dynamic_cast<RoadRunner::DirectJunction*>(target->predecessorJunction.get()) != nullptr
        ||
        std::max(*s1, *s2) == target->Length() &&
        dynamic_cast<RoadRunner::DirectJunction*>(target->successorJunction.get()) != nullptr)
    {
        spdlog::warn("Cannot modify section adjacent to Direct junction. Please delete instead.");
        return true;
    }

    if (target->generated.rr_profile.HasSide(1) != (g_createRoadOption->LeftResult().laneCount > 0) ||
        (target->generated.rr_profile.HasSide(-1) != (g_createRoadOption->RightResult().laneCount > 0)))
    {
        spdlog::warn("Modified road must have some travel direction(s) as before!");
        return true;
    }

    return target->ModifyProfile(*s1, *s2, g_createRoadOption->LeftResult(), g_createRoadOption->RightResult());
}