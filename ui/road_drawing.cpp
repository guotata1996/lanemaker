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

extern std::weak_ptr<RoadRunner::Road> g_PointerRoad;
extern double g_PointerRoadS;

extern SectionProfileConfigWidget* g_createRoadOption;

extern double g_zoom;

RoadDrawingSession::RoadDrawingSession(QGraphicsView* aView) :
    view(aView), scene(aView->scene()), world(World::Instance())
{
    cursorItem = new CustomCursorItem;
    scene->addItem(cursorItem);
    cursorItem->hide(); // Hide until we receive mouse position
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
    return CustomCursorItem::SnapRadiusPx / g_zoom;
}

double RoadDrawingSession::GetAdjustedS() const
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

void RoadDrawingSession::BeginPickingProfile()
{
    beginPickingS = g_PointerRoadS;
    beginPickingRoad = g_PointerRoad;
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
}

double CustomCursorItem::SnapRadiusPx = 20;
double CustomCursorItem::InitialRadius = 2;

void CustomCursorItem::EnableHighlight(bool enable)
{
    setBrush(enable ? QBrush(Qt::red, Qt::SolidPattern) : Qt::NoBrush);
}

void CustomCursorItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    this->setScale(SnapRadiusPx / InitialRadius / g_zoom);
    QGraphicsEllipseItem::paint(painter, option, widget);
}

RoadCreationSession::RoadCreationSession(QGraphicsView* aView) :
    RoadDrawingSession(aView)
{
    setPreviewItem = scene->addPath(setPath);
    setPreviewItem->setZValue(10);

    flexPreviewItem = scene->addPath(flexPath);
    flexPreviewItem->setZValue(10);
    QPen flexPen;
    flexPen.setStyle(Qt::DashLine);
    flexPreviewItem->setPen(flexPen);

    hintItem = scene->addPath(hintPath);
    hintItem->setZValue(10);
    QPen hintPen;
    hintPen.setColor(QColor(0, 200, 100, 80));
    hintPen.setStyle(Qt::DotLine);
    hintItem->setPen(hintPen);

    ctrlPoints.push_back(QPointF());
}

bool RoadCreationSession::Update(QMouseEvent* event)
{
    SetHighlightTo(g_PointerRoad.lock());

    QPointF scenePos = view->mapToScene(event->pos());

    if (event->button() == Qt::MouseButton::LeftButton)
    {
        if (event->type() == QEvent::Type::MouseButtonPress)
        {
            ctrlPoints.push_back(scenePos);
            if (ctrlPoints.size() >= 3 && !joinAtEnd.expired())
            {
                return false;
            }
        }
        else if (event->type() == QEvent::Type::MouseButtonDblClick
            && ctrlPoints.size() > 2
            && ctrlPoints.size() % 2 == 1)
        {
            auto last3Point = ctrlPoints[ctrlPoints.size() - 3];
            auto& last2Point = ctrlPoints[ctrlPoints.size() - 2];
            auto lastPoint = ctrlPoints.back();
            auto midPoint = (lastPoint + last3Point) / 2;
            last2Point.setX(midPoint.x());
            last2Point.setY(midPoint.y());

            ctrlPoints.push_back(scenePos);
        }
    }
    else if (event->button() == Qt::MouseButton::RightButton)
    {
        if (ctrlPoints.size() > 1)
        {
            // At least keep one
            ctrlPoints.pop_back();
        }
        else if (!g_PointerRoad.expired() && event->type() == QEvent::Type::MouseButtonPress)
        {
            BeginPickingProfile();
        }
        else if (event->type() == QEvent::Type::MouseButtonRelease)
        {
            EndPickingProfile();
        }
    }
    else if (event->button() == Qt::MouseButton::NoButton)
    {
        ctrlPoints.back().setX(scenePos.x());
        ctrlPoints.back().setY(scenePos.y());
    }

    if (PickProfileMode())
    {
        ContinuePickingProfile();
    }

    hintPath.clear();
    float maxSnapDist = ctrlPoints.size() % 2 == 1 ? SnapDistFromScale() : 1e9;
    bool onExisting = SnapCtrlPoint(maxSnapDist);
    cursorItem->setPos(ctrlPoints.back());
    cursorItem->EnableHighlight(onExisting);
    cursorItem->show();

    setPath.clear();
    flexPath.clear();

    if (ctrlPoints.size() >= 2)
    {
        if (!ctrlPoints.empty())
        {
            setPath.moveTo(ctrlPoints[0]);
        }
        int i = 1;
        for (; i + 2 < ctrlPoints.size(); i += 2)
        {
            setPath.quadTo(ctrlPoints[i], ctrlPoints[i + 1]);
        }

        auto toJoin = joinAtEnd.lock();
        if (toJoin == nullptr)
        {
            if (i == ctrlPoints.size() - 1)
            {
                flexPath.moveTo(ctrlPoints[i - 1]);
                flexPath.lineTo(ctrlPoints[i]);
            }
            else if (i == ctrlPoints.size() - 2)
            {
                flexPath.moveTo(ctrlPoints[i - 1]);
                flexPath.quadTo(ctrlPoints[i], ctrlPoints[i + 1]);
            }
        }
        else if (ctrlPoints.size() >= 3)
        {
            auto flexGeometry = CreateJoinAtEndGeo(true);
            double flexLen = flexGeometry->length;
            for (int subDiv = 0; subDiv != 51; ++subDiv)
            {
                auto p = flexGeometry->get_xy(flexLen / 50 * subDiv);
                QPointF qp(p[0], p[1]);
                if (subDiv == 0)
                {
                    flexPath.moveTo(qp);
                }
                else
                {
                    flexPath.lineTo(qp);
                }
            }
        }
    }

    setPreviewItem->setPath(setPath);
    flexPreviewItem->setPath(flexPath);
    hintItem->setPath(hintPath);

    return true;
}

void RoadCreationSession::Complete()
{
    Stats s("LaneGraphics Created");
    CreateRoad();
}

RoadCreationSession::~RoadCreationSession()
{
    scene->removeItem(setPreviewItem);
    scene->removeItem(flexPreviewItem);
    scene->removeItem(hintItem);
    scene->removeItem(cursorItem);
    SetHighlightTo(nullptr);
}

bool RoadCreationSession::SnapFirstPointToExisting(QPointF& point)
{
    auto g_road = g_PointerRoad.lock();
    if (g_road == nullptr) return false;

    const double snapThreshold = SnapDistFromScale();
    double snapS = g_PointerRoadS;
    bool onExisting = false;
    if (g_PointerRoadS < snapThreshold &&
        dynamic_cast<RoadRunner::DirectJunction*>(g_PointerRoad.lock()->predecessorJunction.get()) == nullptr)
    {
        snapS = 0;
        auto grad = g_road->generated.ref_line.get_grad_xy(snapS);
        startDir = std::make_unique<QVector2D>(-grad[0], -grad[1]);
        extendFromStart = g_PointerRoad;
        extendFromStartS = 0;
    }
    else if (g_PointerRoadS > g_road->Length() - snapThreshold &&
        dynamic_cast<RoadRunner::DirectJunction*>(g_PointerRoad.lock()->successorJunction.get()) == nullptr)
    {
        snapS = g_road->Length();
        auto grad = g_road->generated.ref_line.get_grad_xy(snapS);
        startDir = std::make_unique<QVector2D>(grad[0], grad[1]);
        extendFromStart = g_PointerRoad;
        extendFromStartS = g_road->Length();
    }

    if (!extendFromStart.expired())
    {
        // only snap to ends
        auto snapped = g_road->generated.ref_line.get_xy(snapS);
        point.setX(snapped[0]);
        point.setY(snapped[1]);
        onExisting = true;
    }
    return onExisting;
}

bool RoadCreationSession::SnapLastPointToExisting(QPointF& point)
{
    auto g_road = g_PointerRoad.lock();
    if (g_road == nullptr) return false;

    bool onExisting = false;

    // Join to existing
    double snapS;
    const double snapThreshold = SnapDistFromScale();
    if (g_PointerRoadS < snapThreshold &&
        dynamic_cast<RoadRunner::DirectJunction*>(g_PointerRoad.lock()->predecessorJunction.get()) == nullptr)
    {
        snapS = 0;
        joinAtEnd = g_PointerRoad;
    }
    else if (g_PointerRoadS > g_road->Length() - snapThreshold &&
        dynamic_cast<RoadRunner::DirectJunction*>(g_PointerRoad.lock()->successorJunction.get()) == nullptr)
    {
        snapS = g_road->Length();
        joinAtEnd = g_PointerRoad;
    }
    if (!joinAtEnd.expired())
    {
        auto snapped = g_road->generated.ref_line.get_xy(snapS);
        point.setX(snapped[0]);
        point.setY(snapped[1]);
        onExisting = true;
        joinAtEndS = snapS;
    }
    return onExisting;
}

bool RoadCreationSession::SnapCtrlPoint(float maxOffset)
{
    QPointF& nextPoint = ctrlPoints.back();

    QVector2D nextDir;
    bool onExisting = false;

    if (ctrlPoints.size() == 1)
    {
        startDir.reset();
        extendFromStart.reset();

        return SnapFirstPointToExisting(nextPoint);
    }
    else if (ctrlPoints.size() == 2)
    {
        // Extend from existing
        if (startDir != nullptr)
        {
            nextDir = *startDir;
        }
    }
    else
    {
        joinAtEnd.reset();
        onExisting = SnapLastPointToExisting(nextPoint);

        if (!onExisting)
        {
            nextDir = QVector2D(ctrlPoints[ctrlPoints.size() - 2] - ctrlPoints[ctrlPoints.size() - 3]);
        }
    }

    if (nextDir.lengthSquared() > 0.01f)
    {
        nextDir.normalize();
        hintPath.moveTo(ctrlPoints[ctrlPoints.size() - 2]);
        hintPath.lineTo(ctrlPoints[ctrlPoints.size() - 2] + QPointF(nextDir.x(), nextDir.y()) * 100);

        // Do projection
        QPointF lastPoint = ctrlPoints[ctrlPoints.size() - 2];
        QVector2D last2Next(nextPoint - lastPoint);
        auto projLength = QVector2D::dotProduct(last2Next, nextDir);
        projLength = std::max(0.0f, projLength);
        QVector2D projected = QVector2D(lastPoint) + projLength * nextDir;
        if (projected.distanceToPoint(QVector2D(nextPoint)) < maxOffset)
        {
            nextPoint.setX(projected.x());
            nextPoint.setY(projected.y());
        }
    }

    return onExisting;
}

odr::RefLine RoadCreationSession::RefLineFromCtrlPoints() const
{
    odr::RefLine rtn("", 0);
    if (!joinAtEnd.expired() && ctrlPoints.size() == 4)
    {
        auto roadGeometry = CreateJoinAtEndGeo(false);
        if (roadGeometry->length > RoadRunner::ValidGeoMaxLength)
        {
            spdlog::warn("Abort create road: Potential invalid shape!");
            return rtn;
        }
        rtn.length = roadGeometry->length;
        rtn.s0_to_geometry.emplace(0, std::move(roadGeometry));
    }
    else if (ctrlPoints.size() >= 4)
    {
        double cumLength = 0;
        // Last ctrl point is a duplicate. Do not use in any case.
        int validCtrlPoints = joinAtEnd.expired() ? ctrlPoints.size() - 1 : ctrlPoints.size() - 2;
        for (int i = 0; i + 2 < validCtrlPoints; i += 2)
        {
            std::unique_ptr<odr::RoadGeometry> localGeometry;

            QVector2D start(ctrlPoints[i]);
            QVector2D middle(ctrlPoints[i + 1]);
            QVector2D end(ctrlPoints[i + 2]);
            odr::Vec2D point1{ start.x(), start.y() };
            odr::Vec2D point2{ middle.x(), middle.y() };
            odr::Vec2D point3{ end.x(), end.y() };

            if (middle.distanceToLine(start, end - start) < 0.1f)
            {
                localGeometry = std::make_unique<odr::Line>(cumLength, point1, point3);
            }
            else
            {
                localGeometry = std::make_unique<odr::ParamPoly3>(cumLength, point1, point2, point2, point3);
            }
            double cumLengthPrev = cumLength;
            cumLength += localGeometry->length;
            rtn.s0_to_geometry.emplace(cumLengthPrev, std::move(localGeometry));
        }
        rtn.length = cumLength;
    }
    return rtn;
}

void RoadCreationSession::CreateRoad()
{
    if (!extendFromStart.expired() && extendFromStart.lock() == joinAtEnd.lock())
    {
        spdlog::warn("Self-loop is not supported!");
        return;
    }
    
    auto refLine = RefLineFromCtrlPoints();
    if (refLine.length == 0)
    {
        spdlog::warn("Not enough control points placed");
        return;
    }

    RoadRunner::RoadProfile config(
        g_createRoadOption->LeftResult().laneCount, g_createRoadOption->LeftResult().offsetx2,
        g_createRoadOption->RightResult().laneCount, g_createRoadOption->RightResult().offsetx2);

    auto newRoad = std::make_shared<RoadRunner::Road>(config, refLine);
    newRoad->GenerateAllSectionGraphics();

    bool standaloneRoad = true;
    // Which part of newRoad will be newly-created?
    double newPartBegin = 0;
    if (!extendFromStart.expired())
    {
        standaloneRoad = false;
        auto toExtend = extendFromStart.lock();
        newPartBegin = toExtend->Length();
        int joinResult = RoadRunner::Road::JoinRoads(toExtend, 
            extendFromStartS == 0 ? odr::RoadLink::ContactPoint_Start : odr::RoadLink::ContactPoint_End,
            newRoad, odr::RoadLink::ContactPoint_Start);
        if (joinResult != 0)
        {
            spdlog::error("Extend error {}", joinResult);
        }

        newRoad = toExtend;
    }

    if (!joinAtEnd.expired())
    {
        if (!standaloneRoad)
        {
            world->allRoads.erase(newRoad);
        }

        standaloneRoad = false;
        auto toJoin = joinAtEnd.lock();
        newPartBegin = toJoin->Length();
        world->allRoads.erase(toJoin);

        int joinResult = RoadRunner::Road::JoinRoads(
            newRoad, odr::RoadLink::ContactPoint_End, 
            toJoin, joinAtEndS == 0 ? odr::RoadLink::ContactPoint_Start : odr::RoadLink::ContactPoint_End);
        if (joinResult != 0)
        {
            spdlog::error("Join error {}", joinResult);
        }
        world->allRoads.insert(newRoad); //TODO: same change for lane mode
    }

    if (standaloneRoad)
    {
        world->allRoads.insert(newRoad);
    }

    tryCreateJunction(std::move(newRoad), newPartBegin);
}

std::unique_ptr<odr::RoadGeometry> RoadCreationSession::CreateJoinAtEndGeo(bool forPreview) const
{
    return createJoinAtEndGeo(forPreview, ForceDirection::None);
}

std::unique_ptr<odr::RoadGeometry> RoadCreationSession::createJoinAtEndGeo(bool forPreview, 
    ForceDirection joinPointDir) const
{
    QPointF p0, p1, p2;
    if (forPreview)
    {
        p0 = ctrlPoints[ctrlPoints.size() - 3];
        p1 = ctrlPoints[ctrlPoints.size() - 2];
        p2 = ctrlPoints[ctrlPoints.size() - 1];
    }
    else
    {
        p0 = ctrlPoints[ctrlPoints.size() - 4];
        p1 = ctrlPoints[ctrlPoints.size() - 3];
        p2 = ctrlPoints[ctrlPoints.size() - 2];
    }
    odr::Vec2D pos0{ p0.x(), p0.y() };
    odr::Vec2D pos1{ p1.x(), p1.y() };
    odr::Vec2D pos2{ p2.x(), p2.y() };
    odr::Vec2D dir0 = odr::normalize(odr::sub(pos1, pos0));

    auto toJoin = joinAtEnd.lock();
    odr::Vec2D dir2 = toJoin->generated.ref_line.get_grad_xy(joinAtEndS);
    if (joinPointDir == ForceDirection::None && joinAtEndS == toJoin->Length()
        || joinPointDir == ForceDirection::Negate)
    {
        dir2 = odr::negate(dir2);
    }

    dir2 = odr::normalize(dir2);

    if (forPreview && ctrlPoints.size() % 2 == 0 || !forPreview && ctrlPoints.size() % 2 == 1)
    {
        // "abruply" join at end
        return RoadRunner::ConnectLines(pos1, dir0, pos2, dir2);
    }
    else
    {
        return RoadRunner::ConnectLines(pos0, dir0, pos2, dir2);
    }
}

void RoadCreationSession::tryCreateJunction(std::shared_ptr<RoadRunner::Road> newRoad, double newPartBegin)
{
    const double JunctionExtaTrim = 10; // Space for connecting road curvature
    const double RoadMinLength = 5; // Discard if any leftover road is too short
    while (true)
    {
        auto overlap = newRoad->FirstOverlap(newPartBegin, newRoad->Length());
        if (overlap == nullptr)
        {
            break;
        }

        auto road2 = overlap->road2.lock();

        bool canCreateJunction = true; // If not, this collision will end up as an overlap
        double sBegin1 = overlap->sBegin1 - JunctionExtaTrim;
        double sEnd1 = overlap->sEnd1 + JunctionExtaTrim;
        double sBegin2 = overlap->sBegin2 - JunctionExtaTrim;
        double sEnd2 = overlap->sEnd2 + JunctionExtaTrim;

        if (sBegin1 < RoadMinLength)
        {
            if (newRoad->predecessorJunction != nullptr)
            {
                // Don't make junction if one is already too close
                canCreateJunction = false;
            }
            else
            {
                sBegin1 = 0; // T junction
            }
        }
        if (sEnd1 > newRoad->Length() - RoadMinLength)
        {
            if (newRoad->successorJunction != nullptr)
            {
                // Don't make junction if one is already too close
                canCreateJunction = false;
            }
            else
            {
                sEnd1 = newRoad->Length();  // T junction
            }
        }

        bool joinExistingJunction = road2->generated.junction != "-1";

        if (!joinExistingJunction)
        {
            if (sBegin2 < RoadMinLength)
            {
                if (road2->predecessorJunction != nullptr)
                {
                    // Don't make junction if one is already too close
                    canCreateJunction = false;
                }
                else
                {
                    sBegin2 = 0;
                }
            }
            if (sEnd2 > road2->Length() - RoadMinLength)
            {
                if (road2->successorJunction != nullptr)
                {
                    // Don't make junction if one is already too close
                    canCreateJunction = false;
                }
                else
                {
                    sEnd2 = road2->Length();
                }
            }

            if (sBegin1 == 0 && sEnd1 == newRoad->Length()
                || sBegin2 == 0 && sEnd2 == road2->Length())
            {
                spdlog::warn("Road is too short! Cannot create Junction with Road{} @{}", road2->ID(), sEnd1);
                newPartBegin = sEnd1 + RoadMinLength;
                continue;
            }
        }

        if (!canCreateJunction)
        {
            spdlog::warn("Existing junction is too close! Cannot create Junction with Road{} @{}", road2->ID(), sEnd1);
            newPartBegin = sEnd1 + RoadMinLength;
            continue;
        }

        // Can create junction
        std::shared_ptr<RoadRunner::Road> newRoadBeforeJunction, newRoadPastJunction;
        
        if (sEnd1 != newRoad->Length())
        {
            newRoadPastJunction = RoadRunner::Road::SplitRoad(newRoad, sEnd1);
            world->allRoads.insert(newRoadPastJunction);
        }
        if (sBegin1 != 0)
        {
            RoadRunner::Road::SplitRoad(newRoad, sBegin1);
            newRoadBeforeJunction = newRoad;
        }
        else
        {
            world->allRoads.erase(newRoad);
        }

        if (joinExistingJunction)
        {
            auto junctionPtr = IDGenerator::ForJunction()->GetByID(road2->generated.junction);
            auto junction = static_cast<RoadRunner::Junction*>(junctionPtr)->shared_from_this();
            if (newRoadBeforeJunction != nullptr)
            {
                junction->Attach(RoadRunner::ConnectionInfo{ newRoadBeforeJunction, odr::RoadLink::ContactPoint_End });
            }
            else if (newRoadPastJunction != nullptr)
            {
                junction->Attach(RoadRunner::ConnectionInfo{ newRoadPastJunction, odr::RoadLink::ContactPoint_Start });
            }
            else
            {
                spdlog::warn("Junctions too close or road too short to join existing junction!");
            }
            // New road beyond junction will be trimmed, unless it starts from the junction
            if (sBegin1 != 0 && newRoadPastJunction != nullptr)
            {
                world->allRoads.erase(newRoadPastJunction);
                break;
            }
        }
        else
        {
            std::shared_ptr<RoadRunner::Road> road2BeforeJunction, road2PastJunction;
            if (sEnd2 != road2->Length())
            {
                road2PastJunction = RoadRunner::Road::SplitRoad(road2, sEnd2);
                world->allRoads.insert(road2PastJunction);
            }
            if (sBegin2 != 0)
            {
                RoadRunner::Road::SplitRoad(road2, sBegin2);
                road2BeforeJunction = road2;
            }
            else
            {
                world->allRoads.erase(road2);
            }

            std::vector<RoadRunner::ConnectionInfo> junctionInfo;

            if (newRoadBeforeJunction != nullptr)
            {
                junctionInfo.push_back(RoadRunner::ConnectionInfo{ newRoadBeforeJunction, odr::RoadLink::ContactPoint_End });
            }
            if (newRoadPastJunction != nullptr)
            {
                junctionInfo.push_back(RoadRunner::ConnectionInfo{ newRoadPastJunction, odr::RoadLink::ContactPoint_Start });
            }
            if (road2BeforeJunction != nullptr)
            {
                junctionInfo.push_back(RoadRunner::ConnectionInfo{ road2BeforeJunction, odr::RoadLink::ContactPoint_End });
            }
            if (road2PastJunction != nullptr)
            {
                junctionInfo.push_back(RoadRunner::ConnectionInfo{ road2PastJunction, odr::RoadLink::ContactPoint_Start });
            }
            if (junctionInfo.size() < 3)
            {
                // 2-road junction, should really be a Join
                canCreateJunction = false;
            }
            auto junction = std::make_shared<RoadRunner::Junction>();
            junction->CreateFrom(junctionInfo);
        }

        if (newRoadPastJunction == nullptr)
        {
            break;
        }

        newRoad = newRoadPastJunction;
        newPartBegin = 0;
    }
}