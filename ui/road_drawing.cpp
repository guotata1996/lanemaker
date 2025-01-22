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
#include "map_view_gl.h"

extern SectionProfileConfigWidget* g_createRoadOption;

std::shared_ptr<RoadRunner::Road> RoadDrawingSession::GetPointerRoad()
{
    if (RoadRunner::g_PointerRoadID.empty())
    {
        return std::shared_ptr<RoadRunner::Road>();
    }
    auto g_road = IDGenerator::ForRoad()->GetByID(RoadRunner::g_PointerRoadID);
    if (g_road == nullptr) return std::shared_ptr<RoadRunner::Road>();
    return static_cast<RoadRunner::Road*>(g_road)->shared_from_this();
}

RoadDrawingSession::RoadDrawingSession() :
    world(World::Instance())
{
    cursorItem = std::make_unique<CustomCursorItem>();
}

RoadDrawingSession::~RoadDrawingSession()
{
    SetHighlightTo(nullptr);
}

bool RoadDrawingSession::Update(const RoadRunner::MouseAction& evt)
{
    if (evt.button == Qt::RightButton)
    {
        if (GetPointerRoad() != nullptr &&
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

bool RoadDrawingSession::Update(const RoadRunner::KeyPressAction& evt)
{
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
    return RoadRunner::SnapRadiusPx / std::min(1.0f, RoadRunner::g_mapViewGL->Zoom());
}

double RoadDrawingSession::GetAdjustedS(bool* onSegmentBoundary) const
{
    auto g_road = GetPointerRoad();
    const double snapThreshold = SnapDistFromScale();
    if (RoadRunner::g_PointerRoadS < snapThreshold)
    {
        if (onSegmentBoundary != nullptr) *onSegmentBoundary = true;
        return 0;
    }
    else if (RoadRunner::g_PointerRoadS > g_road->Length() - snapThreshold)
    {
        if (onSegmentBoundary != nullptr) *onSegmentBoundary = true;
        return g_road->Length();
    }
    return g_road->SnapToSegmentBoundary(RoadRunner::g_PointerRoadS, snapThreshold, onSegmentBoundary);
}

void RoadDrawingSession::BeginPickingProfile()
{
    beginPickingS = RoadRunner::g_PointerRoadS;
    beginPickingRoad = GetPointerRoad();
    QPixmap p = QPixmap(":/icons/eyedropper.svg");
    RoadRunner::g_mapViewGL->setCursor(QCursor(p));
}

void RoadDrawingSession::ContinuePickingProfile()
{
    if (beginPickingRoad.expired() || GetPointerRoad() != beginPickingRoad.lock())
    {
        return;
    }
    
    auto pickFromRoad = beginPickingRoad.lock();
    auto leftProfile = pickFromRoad->generated.rr_profile.ProfileAt(beginPickingS, 1);
    auto rightProfile = pickFromRoad->generated.rr_profile.ProfileAt(beginPickingS, -1);
    if (RoadRunner::g_PointerRoadS < beginPickingS)
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
    RoadRunner::g_mapViewGL->setCursor(Qt::ArrowCursor);
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

RoadDrawingSession::CustomCursorItem::CustomCursorItem(): color(Qt::white)
{
}

RoadDrawingSession::CustomCursorItem::~CustomCursorItem()
{
    if (graphicsIndex.has_value())
    {
        RoadRunner::g_mapViewGL->RemoveItem(*graphicsIndex, true);
    }
}

void RoadDrawingSession::CustomCursorItem::EnableHighlight(int level)
{
    color = level == 0 ? Qt::white : (level == 1 ? Qt::darkRed : Qt::red);
    SetTranslation(translation); // remove then add
}

void RoadDrawingSession::CustomCursorItem::SetTranslation(odr::Vec3D t)
{
    translation = t;
    if (graphicsIndex.has_value())
    {
        RoadRunner::g_mapViewGL->RemoveItem(*graphicsIndex, true);
    }
    auto centerOnXY = odr::Vec3D{ t[0], t[1], 0 };
    auto h = t[2];
    
    odr::Line3D roundBoundary;
    for (int i = 0; i < 24; ++i)
    {
        double angle = 2 * M_PI / 24 * i;
        auto offset = odr::Vec3D{ std::cos(angle), std::sin(angle), 0.01 };
        auto pos = odr::add(centerOnXY, offset);
        roundBoundary.push_back(pos);
    }

    if (h == 0)
    {
        graphicsIndex = RoadRunner::g_mapViewGL->AddPoly(roundBoundary, color);
    }
    else
    {
        graphicsIndex = RoadRunner::g_mapViewGL->AddColumn(roundBoundary, h, color);
    }
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
