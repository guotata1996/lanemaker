#include "road_drawing.h"

#include <qevent.h>
#include "CreateRoadOptionWidget.h"

extern std::weak_ptr<RoadRunner::Road> g_PointerRoad;
extern double g_PointerRoadS;

extern CreateRoadOptionWidget* g_createRoadOption;

RoadModificationSession::RoadModificationSession(QGraphicsView* aView) : RoadDestroySession(aView)
{}

bool RoadModificationSession::Update(QMouseEvent* evt)
{
    RoadDestroySession::Update(evt);

    if (evt->button() == Qt::RightButton && !g_PointerRoad.expired())
    {
        // Pick road profile
        auto pickFromRoad = g_PointerRoad.lock();
        auto leftProfile = pickFromRoad->generated.rr_profile.ProfileAt(g_PointerRoadS, 1);
        auto rightProfile = pickFromRoad->generated.rr_profile.ProfileAt(g_PointerRoadS, -1);
        g_createRoadOption->SetOption(leftProfile, rightProfile);
    }
    return true;
}

void RoadModificationSession::Complete()
{
    if (targetRoad.expired()) return;

    targetRoad.lock()->ModifyProfile(*s1, *s2,
        g_createRoadOption->LeftResult(), g_createRoadOption->RightResult());
}