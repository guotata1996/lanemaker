#include "road_drawing.h"

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

std::shared_ptr<LM::Road> RoadDrawingSession::GetPointerRoad()
{
    if (LM::g_PointerRoadID.empty())
    {
        return std::shared_ptr<LM::Road>();
    }
    auto g_road = IDGenerator::ForType(IDType::Road)->GetByID<LM::Road>(LM::g_PointerRoadID);
    if (g_road == nullptr) return std::shared_ptr<LM::Road>();
    return g_road->shared_from_this();
}

odr::Vec2D RoadDrawingSession::CursorAtHeight(double zLevel)
{
    double cameraZ = LM::g_CameraPosition[2];
    double portion = (cameraZ - zLevel) / cameraZ;
    odr::Vec2D cameraXY{ LM::g_CameraPosition[0], LM::g_CameraPosition[1] };
    auto offsetAtGround = odr::sub(LM::g_PointerOnGround, cameraXY);
    auto offsetAtLevel = odr::mut(portion, offsetAtGround);
    return odr::add(cameraXY, offsetAtLevel);
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

bool RoadDrawingSession::Update(const LM::MouseAction& evt)
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

bool RoadDrawingSession::Update(const LM::KeyPressAction& evt)
{
    return true;
}

void RoadDrawingSession::SetHighlightTo(std::shared_ptr<LM::Road> target)
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
    return LM::SnapRadiusStd / std::min(1.0f, LM::g_mapViewGL->Zoom());
}

double RoadDrawingSession::GetAdjustedS(bool* onSegmentBoundary) const
{
    auto g_road = GetPointerRoad();
    const double snapThreshold = SnapDistFromScale();
    if (LM::g_PointerRoadS < snapThreshold)
    {
        if (onSegmentBoundary != nullptr) *onSegmentBoundary = true;
        return 0;
    }
    else if (LM::g_PointerRoadS > g_road->Length() - snapThreshold)
    {
        if (onSegmentBoundary != nullptr) *onSegmentBoundary = true;
        return g_road->Length();
    }
    return g_road->SnapToSegmentBoundary(LM::g_PointerRoadS, snapThreshold, onSegmentBoundary);
}

void RoadDrawingSession::BeginPickingProfile()
{
    beginPickingS = LM::g_PointerRoadS;
    beginPickingRoad = GetPointerRoad();
    QPixmap p = QPixmap(":/icons/eyedropper.svg");
    LM::g_mapViewGL->setCursor(QCursor(p));
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
    if (LM::g_PointerRoadS < beginPickingS)
    {
        std::swap(leftProfile, rightProfile);
        leftProfile.offsetx2 = -leftProfile.offsetx2;
        rightProfile.offsetx2 = -rightProfile.offsetx2;
    }
    g_createRoadOption->SetOption(leftProfile, rightProfile);

    auto elevation = pickFromRoad->generated.ref_line.elevation_profile.get(LM::g_PointerRoadS);
    LM::g_createRoadElevationOption = std::round(elevation / LM::ElevationStep);
}

void RoadDrawingSession::EndPickingProfile()
{
    beginPickingRoad.reset();
    LM::g_mapViewGL->setCursor(Qt::ArrowCursor);
}

bool RoadDrawingSession::IsProfileChangePoint(const std::shared_ptr<LM::Road>& road, double s)
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

RoadDrawingSession::CustomCursorItem::CustomCursorItem(): highlightLevel(0)
{
    DrawGroundGrids();
}

RoadDrawingSession::CustomCursorItem::~CustomCursorItem()
{

}

void RoadDrawingSession::CustomCursorItem::EnableHighlight(int level)
{
    highlightLevel = level;
    SetTranslation(translation); // remove then add
}

void RoadDrawingSession::CustomCursorItem::SetTranslation(odr::Vec3D t)
{
    translation = t;

    Clear();

    auto centerOnXY = odr::Vec3D{ t[0], t[1], 0 };
    auto h = t[2];
    {
        odr::Line3D diskBoundary;
        for (int i = 0; i < 24; ++i)
        {
            double angle = 2 * M_PI / 24 * i;
            auto offset = odr::Vec3D{ std::cos(angle), std::sin(angle), h + 0.05 };
            auto pos = odr::add(centerOnXY, offset);
            diskBoundary.push_back(pos);
        }
        QColor color = highlightLevel == 0 ?
            (h == 0 ? Qt::cyan : Qt::white) : 
            (highlightLevel == 1 ? Qt::darkRed : Qt::red);

        AddPoly(diskBoundary, color);
    }

    if (h != 0)
    {
        odr::Line3D pillarBoundary;
        for (int i = 0; i < 12; ++i)
        {
            double angle = 2 * M_PI / 12 * i;
            auto offset = odr::Vec3D{ std::cos(angle), std::sin(angle), 0 };
            auto pos = odr::add(centerOnXY, odr::mut(0.2, offset));
            pillarBoundary.push_back(pos);
        }
        auto pillarColor = h > 0 ? Qt::green : Qt::darkYellow;
        AddPoly(pillarBoundary, pillarColor, h);
    }
}

void RoadDrawingSession::CustomCursorItem::DrawGroundGrids()
{
    // Draw static ground grids
    for (int x = -50; x <= 50; ++x)
    {
        odr::Line3D l;
        l.push_back(odr::Vec3D{ -2500, x * 50.0, -0.05 });
        l.push_back(odr::Vec3D{ 2500, x * 50.0, -0.05 });
        groundGrids.AddLine(l, 0.25, Qt::black);
    }
    for (int y = -50; y <= 50; ++y)
    {
        odr::Line3D l;
        l.push_back(odr::Vec3D{ y * 50.0, -2500, -0.05 });
        l.push_back(odr::Vec3D{ y * 50.0, 2500, -0.05 });
        groundGrids.AddLine(l, 0.25, Qt::black);
    }
}


void RoadDrawingSession::UpdateEndMarkings()
{
    std::set<std::pair< LM::Road*, odr::RoadLink::ContactPoint>> dueUpdate;
    for (auto id_obj : IDGenerator::ForType(IDType::Junction)->PeekChanges())
    {
        auto junction = static_cast<LM::AbstractJunction*>(id_obj.second);
        if (junction != nullptr)
        {
            auto connected = junction->GetConnected();
            dueUpdate.insert(connected.begin(), connected.end());
        }
    }
    for (auto id_obj : IDGenerator::ForType(IDType::Road)->PeekChanges())
    {
        auto road = static_cast<LM::Road*>(id_obj.second);
        if (road != nullptr)
        {
            //road->EnableHighlight(false);
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
                    bool isCommon = dynamic_cast<LM::Junction*>(road->successorJunction.get()) != nullptr;
                    if (isCommon)
                    {
                        needStopLine = road->generated.rr_profile.HasSide(-1);
                    }

                    for (auto lane : road->generated.s_to_lanesection.rbegin()->second.get_sorted_driving_lanes(-1))
                    {
                        auto turnSemantics = road->successorJunction->GetTurningSemanticsForIncoming(road->ID(), lane.id);
                        laneToArrow.emplace(lane.id, isCommon ? turnSemantics : (turnSemantics == LM::DeadEnd ? LM::DeadEnd : 0));
                    }
                }
                else
                {
                    for (auto lane : road->generated.s_to_lanesection.rbegin()->second.get_sorted_driving_lanes(-1))
                    {
                        laneToArrow.emplace(lane.id, LM::DeadEnd);
                    }
                }
            }
            else
            {
                if (road->predecessorJunction != nullptr)
                {
                    bool isCommon = dynamic_cast<LM::Junction*>(road->predecessorJunction.get()) != nullptr;
                    if (isCommon)
                    {
                        needStopLine = road->generated.rr_profile.HasSide(1);
                    }

                    for (auto lane : road->generated.s_to_lanesection.begin()->second.get_sorted_driving_lanes(1))
                    {
                        auto turnSemantics = road->predecessorJunction->GetTurningSemanticsForIncoming(road->ID(), lane.id);
                        laneToArrow.emplace(lane.id, isCommon ? turnSemantics : (turnSemantics == LM::DeadEnd ? LM::DeadEnd : 0));
                    }
                }
                else
                {
                    for (auto lane : road->generated.s_to_lanesection.begin()->second.get_sorted_driving_lanes(1))
                    {
                        laneToArrow.emplace(lane.id, LM::DeadEnd);
                    }
                }
            }
        }
        road->UpdateArrowGraphics(road_contact.second, laneToArrow, needStopLine);
    }
}
