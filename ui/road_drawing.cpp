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

RoadDrawingSession::~RoadDrawingSession()
{
    SetHighlightTo(nullptr);
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
    return true;
}

bool RoadDrawingSession::IsProfileChangePoint(const std::shared_ptr<RoadRunner::Road>& road, double s)
{
    bool leftOffsetChange = false, rightOffsetChange = false;
    const auto& profile = road->generated.rr_profile;
    if (profile.HasSide(1))
    {
        leftOffsetChange = profile.ProfileAt(s + 0.01, 1).offsetx2 !=
            profile.ProfileAt(s - 0.01, 1).offsetx2;
    }
    if (profile.HasSide(-1))
    {
        rightOffsetChange = profile.ProfileAt(s - 0.01, -1).offsetx2 !=
            profile.ProfileAt(s + 0.01, -1).offsetx2;
    }
    return leftOffsetChange || rightOffsetChange;
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

void RoadDrawingSession::UpdateEndMarkings()
{
    std::set<std::pair< RoadRunner::Road*, odr::RoadLink::ContactPoint>> dueUpdate;
    for (auto id_obj : IDGenerator::ForJunction()->PeekChanges())
    {
        auto junction = static_cast<RoadRunner::AbstractJunction*>(id_obj.second);
        if (junction != nullptr)
        {
            auto connected = junction->GetConnected();
            dueUpdate.insert(connected.begin(), connected.end());
        }
    }
    for (auto id_obj : IDGenerator::ForRoad()->PeekChanges())
    {
        auto road = static_cast<RoadRunner::Road*>(id_obj.second);
        if (road != nullptr)
        {
            road->EnableHighlight(false);
            dueUpdate.insert(std::make_pair(road, odr::RoadLink::ContactPoint_Start));
            dueUpdate.insert(std::make_pair(road, odr::RoadLink::ContactPoint_End));
        }
    }

    for (auto road_contact : dueUpdate)
    {
        auto road = road_contact.first;
        bool needStopLine = false;
        std::map<int, uint8_t> laneToArrow;
        if (road->generated.junction == "-1")
        {
            // Nothing is needed for connecting road
            if (road_contact.second == odr::RoadLink::ContactPoint_End)
            {
                if (road->successorJunction != nullptr)
                {
                    bool isCommon = dynamic_cast<RoadRunner::Junction*>(road->successorJunction.get()) != nullptr;
                    if (isCommon)
                    {
                        needStopLine = road->generated.rr_profile.HasSide(-1);
                    }

                    for (auto lane : road->generated.s_to_lanesection.rbegin()->second.get_sorted_driving_lanes(-1))
                    {
                        auto turnSemantics = road->successorJunction->GetTurningSemanticsForIncoming(road->ID(), lane.id);
                        laneToArrow.emplace(lane.id, isCommon ? turnSemantics : (turnSemantics == RoadRunner::DeadEnd ? RoadRunner::DeadEnd : 0));
                    }
                }
                else
                {
                    for (auto lane : road->generated.s_to_lanesection.rbegin()->second.get_sorted_driving_lanes(-1))
                    {
                        laneToArrow.emplace(lane.id, RoadRunner::DeadEnd);
                    }
                }
            }
            else
            {
                if (road->predecessorJunction != nullptr)
                {
                    bool isCommon = dynamic_cast<RoadRunner::Junction*>(road->predecessorJunction.get()) != nullptr;
                    if (isCommon)
                    {
                        needStopLine = road->generated.rr_profile.HasSide(1);
                    }

                    for (auto lane : road->generated.s_to_lanesection.begin()->second.get_sorted_driving_lanes(1))
                    {
                        auto turnSemantics = road->predecessorJunction->GetTurningSemanticsForIncoming(road->ID(), lane.id);
                        laneToArrow.emplace(lane.id, isCommon ? turnSemantics : (turnSemantics == RoadRunner::DeadEnd ? RoadRunner::DeadEnd : 0));
                    }
                }
                else
                {
                    for (auto lane : road->generated.s_to_lanesection.begin()->second.get_sorted_driving_lanes(1))
                    {
                        laneToArrow.emplace(lane.id, RoadRunner::DeadEnd);
                    }
                }
            }
        }
        road->UpdateArrowGraphics(road_contact.second, laneToArrow, needStopLine);
    }
}
double RoadDrawingSession::CustomCursorItem::InitialRadius = 2;
