#include "road_drawing.h"

#include <QGraphicsPathItem>
#include <qevent.h>

#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
#include "curve_fitting.h"
#include "junction.h"
#include "CreateRoadOptionWidget.h"
#include "stats.h"
#include "constants.h"
#include "map_view.h"

extern std::weak_ptr<RoadRunner::Road> g_PointerRoad;
extern double g_PointerRoadS;
extern int g_RotatingIndex;
extern int g_RotatingSize;

extern SectionProfileConfigWidget* g_createRoadOption;
extern int8_t g_createRoadElevationOption;

extern MapView* g_mapView;

RoadDrawingSession::RoadDrawingSession(QGraphicsView* aView) :
    view(aView), scene(aView->scene()), world(World::Instance())
{
    cursorItem = new CustomCursorItem;
    scene->addItem(cursorItem);
    cursorItem->hide(); // Hide until we receive mouse position
}

bool RoadDrawingSession::Update(const RoadRunner::MouseAction& evt)
{
    if (evt.button == Qt::RightButton)
    {
        if (!g_PointerRoad.expired() &&
            evt.type == QEvent::Type::MouseButtonPress)
        {
            BeginPickingProfile();
        }
        else if (evt.type == QEvent::Type::MouseButtonRelease)
        {
            EndPickingProfile();
        }
    }
    
    if (PickProfileMode())
    {
        ContinuePickingProfile();
    }
    return true;
}

void RoadDrawingSession::SetHighlightTo(std::shared_ptr<RoadRunner::Road> target)
{
    auto currHighlighted = highlighted.lock();
    if (target == currHighlighted) return;
    
    if (currHighlighted != nullptr)
    {
        currHighlighted->EnableHighlight(false);
    }
    if (target != nullptr)
    {
        target->EnableHighlight(true);
    }
    
    highlighted = target;
}

float RoadDrawingSession::SnapDistFromScale() const
{
    return RoadRunner::SnapRadiusPx / g_mapView->Zoom();
}

double RoadDrawingSession::GetAdjustedS(bool* onSegmentBoundary) const
{
    auto g_road = g_PointerRoad.lock();
    const double snapThreshold = SnapDistFromScale();
    if (g_PointerRoadS < snapThreshold)
    {
        if (onSegmentBoundary != nullptr) *onSegmentBoundary = true;
        return 0;
    }
    else if (g_PointerRoadS > g_road->Length() - snapThreshold)
    {
        if (onSegmentBoundary != nullptr) *onSegmentBoundary = true;
        return g_road->Length();
    }
    return g_road->SnapToSegmentBoundary(g_PointerRoadS, snapThreshold, onSegmentBoundary);
}

void RoadDrawingSession::BeginPickingProfile()
{
    beginPickingS = g_PointerRoadS;
    beginPickingRoad = g_PointerRoad;
    QPixmap p = QPixmap(":/icons/eyedropper.svg");
    view->setCursor(QCursor(p));
}

void RoadDrawingSession::ContinuePickingProfile()
{
    if (beginPickingRoad.expired() || g_PointerRoad.lock() != beginPickingRoad.lock())
    {
        return;
    }
    
    auto pickFromRoad = beginPickingRoad.lock();
    auto leftProfile = pickFromRoad->generated.rr_profile.ProfileAt(beginPickingS, 1);
    auto rightProfile = pickFromRoad->generated.rr_profile.ProfileAt(beginPickingS, -1);
    if (g_PointerRoadS < beginPickingS)
    {
        std::swap(leftProfile, rightProfile);
        leftProfile.offsetx2 = -leftProfile.offsetx2;
        rightProfile.offsetx2 = -rightProfile.offsetx2;
    }
    g_createRoadOption->SetOption(leftProfile, rightProfile);
}

void RoadDrawingSession::EndPickingProfile()
{
    beginPickingRoad.reset();
    view->setCursor(Qt::ArrowCursor);
}

bool RoadDrawingSession::IsElevationConsistWithExtend()
{
    switch (g_createRoadElevationOption)
    {
    case 1:
        return g_RotatingIndex == 0;
    case -1:
        return g_RotatingIndex == g_RotatingSize - 1;
    default:
        return g_RotatingSize == 1;
    }
}

void RoadDrawingSession::CustomCursorItem::EnableHighlight(int level)
{
    setBrush(level > 0 ? QBrush(level == 1 ? Qt::darkRed : Qt::red, Qt::SolidPattern) : Qt::NoBrush);
}

void RoadDrawingSession::CustomCursorItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    this->setScale(RoadRunner::SnapRadiusPx / InitialRadius / g_mapView->Zoom());
    QGraphicsEllipseItem::paint(painter, option, widget);
}

double RoadDrawingSession::CustomCursorItem::InitialRadius = 2;
