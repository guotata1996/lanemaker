#include "road_drawing.h"

#include <qevent.h>
#include "CreateRoadOptionWidget.h"

extern std::weak_ptr<RoadRunner::Road> g_PointerRoad;
extern double g_PointerRoadS;

extern SectionProfileConfigWidget* g_createRoadOption;

RoadModificationSession::RoadModificationSession(QGraphicsView* aView) : RoadDestroySession(aView)
{}

bool RoadModificationSession::Update(QMouseEvent* evt)
{
    RoadDestroySession::Update(evt);

    if (!g_PointerRoad.expired() && evt->type() == QEvent::Type::MouseButtonPress)
    {
        BeginPickingProfile();
    }
    else if (evt->type() == QEvent::Type::MouseButtonRelease)
    {
        EndPickingProfile();
    }
    if (PickProfileMode())
    {
        ContinuePickingProfile();
    }
    return true;
}

void RoadModificationSession::Complete()
{
    if (targetRoad.expired()) return;

    targetRoad.lock()->ModifyProfile(*s1, *s2,
        g_createRoadOption->LeftResult(), g_createRoadOption->RightResult());
}