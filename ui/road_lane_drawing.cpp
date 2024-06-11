#include "road_drawing.h"

#include <qvector2d.h>
#include <qevent.h>

#include "CreateRoadOptionWidget.h"
#include "junction.h"

extern std::weak_ptr<RoadRunner::Road> g_PointerRoad;
extern int g_PointerLane;
extern SectionProfileConfigWidget* g_createRoadOption;

LanesCreationSession::LanesCreationSession(QGraphicsView* aView) :
    RoadCreationSession(aView), lLanes(0), rLanes(0)
{

}

void LanesCreationSession::CreateRoad()
{
    if (!extendFromStart.expired() && extendFromStart.lock() == joinAtEnd.lock())
    {
        spdlog::warn("Self-loop is not supported!");
        return;
    }

    if (extendFromStart.expired() && joinAtEnd.expired())
    {
        // Standalone road
        RoadCreationSession::CreateRoad();
        return;
    }

    auto refLine = RefLineFromCtrlPoints();
    if (!joinAtEnd.expired() && ctrlPoints.size() > 4)
    {
        auto lastPiece = CreateJoinAtEndGeo(false);
        auto lastPieceLength = lastPiece->length;
        lastPiece->s0 = refLine.length;
        refLine.s0_to_geometry.emplace(refLine.length, std::move(lastPiece));
        refLine.length += lastPieceLength;
    }
    if (refLine.length == 0)
    {
        spdlog::warn("Not enough control points placed");
        return;
    }
    RoadRunner::RoadProfile config;
    config = RoadRunner::RoadProfile(lLanes, lOffsetX2, rLanes, rOffsetX2);

    auto newRoad = std::make_shared<RoadRunner::Road>(config, refLine);
    newRoad->GenerateAllSectionGraphics();

    bool standaloneRoad = true;
    if (!extendFromStart.expired())
    {
        auto toExtend = extendFromStart.lock();
        RoadRunner::ConnectionInfo linkedInfo(newRoad, odr::RoadLink::ContactPoint_Start, startLanesSkip);
        if (extendFromStartS == 0 || extendFromStartS == toExtend->Length())
        {
            std::shared_ptr<RoadRunner::AbstractJunction> directJunction;
            if (extendFromStartS == 0 && toExtend->predecessorJunction != nullptr)
            {
                directJunction = toExtend->predecessorJunction;
            }
            else if (extendFromStartS == toExtend->Length() && toExtend->successorJunction != nullptr)
            {
                directJunction = toExtend->successorJunction;
            }

            if (directJunction != nullptr)
            {
                directJunction->Attach(linkedInfo);
            }
            else if (startFullyMatch)
            {
                standaloneRoad = false;
                int joinResult = RoadRunner::Road::JoinRoads(toExtend,
                    extendFromStartS == 0 ? odr::RoadLink::ContactPoint_Start : odr::RoadLink::ContactPoint_End,
                    newRoad, odr::RoadLink::ContactPoint_Start);
                if (joinResult != 0)
                {
                    spdlog::error("Extend error {}", joinResult);
                }

                newRoad = toExtend;
            }
            else
            {
                if (extendFromStartS == 0)
                {
                    directJunction = std::make_shared<RoadRunner::DirectJunction>(
                        RoadRunner::ConnectionInfo(toExtend, odr::RoadLink::ContactPoint_Start));
                }
                else
                {
                    directJunction = std::make_shared<RoadRunner::DirectJunction>(
                        RoadRunner::ConnectionInfo(toExtend, odr::RoadLink::ContactPoint_End));
                }
                directJunction->Attach(linkedInfo);
            }
        }
        else
        {
            auto secondHalf = RoadRunner::Road::SplitRoad(toExtend, extendFromStartS);
            world->allRoads.insert(secondHalf);

            std::shared_ptr<RoadRunner::AbstractJunction> directJunction;
            if (startSide < 0)
            {
                directJunction = std::make_shared< RoadRunner::DirectJunction>(
                    RoadRunner::ConnectionInfo(toExtend, odr::RoadLink::ContactPoint_End));
                directJunction->Attach(RoadRunner::ConnectionInfo(secondHalf, odr::RoadLink::ContactPoint_Start, startSplitOffset));
            }
            else
            {
                directJunction = std::make_shared< RoadRunner::DirectJunction>(
                    RoadRunner::ConnectionInfo(secondHalf, odr::RoadLink::ContactPoint_Start));
                directJunction->Attach(RoadRunner::ConnectionInfo(toExtend, odr::RoadLink::ContactPoint_End, startSplitOffset));
            }
            directJunction->Attach(linkedInfo);
        }
    }
    
    if (!joinAtEnd.expired())
    {
        auto toJoin = joinAtEnd.lock();
        RoadRunner::ConnectionInfo linkedInfo(newRoad, odr::RoadLink::ContactPoint_End, endLanesSkip);
        if (joinAtEndS == 0 || joinAtEndS == toJoin->Length())
        {
            std::shared_ptr<RoadRunner::AbstractJunction> directJunction;
            if (joinAtEndS == 0 && toJoin->predecessorJunction != nullptr)
            {
                directJunction = toJoin->predecessorJunction;
            }
            else if (joinAtEndS == toJoin->Length() && toJoin->successorJunction != nullptr)
            {
                directJunction = toJoin->successorJunction;
            }

            if (directJunction != nullptr)
            {
                directJunction->Attach(linkedInfo);
            }
            else if (endFullyMatch)
            {
                standaloneRoad = false;
                world->allRoads.erase(toJoin);
                int joinResult = RoadRunner::Road::JoinRoads(newRoad, odr::RoadLink::ContactPoint_End,
                    toJoin, joinAtEndS == 0 ? odr::RoadLink::ContactPoint_Start : odr::RoadLink::ContactPoint_End);
                if (joinResult != 0)
                {
                    spdlog::error("Join error {}", joinResult);
                }
                world->allRoads.insert(newRoad);
            }
            else
            {
                if (joinAtEndS == 0)
                {
                    directJunction = std::make_shared<RoadRunner::DirectJunction>(
                        RoadRunner::ConnectionInfo(toJoin, odr::RoadLink::ContactPoint_Start));
                }
                else
                {
                    directJunction = std::make_shared<RoadRunner::DirectJunction>(
                        RoadRunner::ConnectionInfo(toJoin, odr::RoadLink::ContactPoint_End));
                }
                directJunction->Attach(linkedInfo);
            }
        }
        else
        {
            auto secondHalf = RoadRunner::Road::SplitRoad(toJoin, joinAtEndS);
            world->allRoads.insert(secondHalf);

            std::shared_ptr<RoadRunner::AbstractJunction> directJunction;
            if (endSide < 0)
            {
                directJunction = std::make_shared< RoadRunner::DirectJunction>(
                    RoadRunner::ConnectionInfo(secondHalf, odr::RoadLink::ContactPoint_Start));
                directJunction->Attach(RoadRunner::ConnectionInfo(toJoin, odr::RoadLink::ContactPoint_End, endSplitOffset));
            }
            else
            {
                directJunction = std::make_shared<RoadRunner::DirectJunction>(
                    RoadRunner::ConnectionInfo(toJoin, odr::RoadLink::ContactPoint_End));
                directJunction->Attach(RoadRunner::ConnectionInfo(secondHalf, odr::RoadLink::ContactPoint_Start, endSplitOffset));
            }
            directJunction->Attach(linkedInfo);
        }
    }
    
    if (standaloneRoad)
    {
        world->allRoads.insert(newRoad);
    }
}

LanesCreationSession::~LanesCreationSession()
{
    SetHighlightTo(nullptr);
}

bool LanesCreationSession::SnapFirstPointToExisting(QPointF& point)
{
    auto g_road = g_PointerRoad.lock();
    if (g_road == nullptr)
    {
        return false;
    }

    const auto rightQuery = g_createRoadOption->RightResult();
    const auto leftQuery = g_createRoadOption->LeftResult();
    
    rLanes = rightQuery.laneCount;
    lLanes = leftQuery.laneCount;

    if (rLanes == 0)
    {
        spdlog::warn("Right lanes must > 0");
        return false;
    }

    double g_roadS = GetAdjustedS();
    const auto& g_roadProfile = g_road->generated.rr_profile;
    const auto rightProfile = g_roadProfile.ProfileAt(g_roadS, -1);
    const auto leftProfile = g_roadProfile.ProfileAt(g_roadS, 1);
    auto grad = g_road->generated.ref_line.get_grad_xy(g_roadS);
    QVector2D g_dir(grad[0], grad[1]);
    g_dir.normalize();
    startFullyMatch = false;
    startLanesSkip = 0;
    startSplitOffset = 0;

    if (lLanes != 0)
    {
        // Bi-directional ramp
        if (g_roadS == 0 || g_roadS == g_road->Length())
        {
            int actualLeftLanes = g_roadS == 0 ? rightProfile.laneCount : leftProfile.laneCount;
            int actualRightLanes = g_roadS == 0 ? leftProfile.laneCount : rightProfile.laneCount;

            if (actualRightLanes >= rLanes
                && actualLeftLanes >= lLanes)
            {
                if (g_roadS == 0)
                {
                    rOffsetX2 = -leftProfile.offsetx2;
                    lOffsetX2 = -rightProfile.offsetx2;
                }
                else
                {
                    rOffsetX2 = rightProfile.offsetx2;
                    lOffsetX2 = leftProfile.offsetx2;
                }
                auto snapPos = g_road->generated.get_xyz(g_roadS, 0, 0);
                point.setX(snapPos[0]);
                point.setY(snapPos[1]);

                extendFromStart = g_road;
                if (g_roadS == 0) g_dir = -g_dir;
                startFullyMatch = actualRightLanes == rLanes && actualLeftLanes == lLanes;
            }
        }
    }
    else
    {
        // Single-directional ramp
        auto lanesRequired = rLanes;  // New road follows drawing direction
        std::vector<std::pair<uint8_t, uint8_t>> searchRanges;
        double baseOffset;
        startSide = g_PointerLane < 0 ? -1 : 1;
        if (g_roadS == g_road->Length())
        {
            if (g_PointerLane < 0 && rightProfile.laneCount >= lanesRequired)
            {
                searchRanges.push_back(std::make_pair(1, rightProfile.laneCount));
                baseOffset = static_cast<double>(rightProfile.offsetx2) / 2;
                startFullyMatch = rightProfile.laneCount == lanesRequired && leftProfile.laneCount == 0;
            }
        }
        else if (g_roadS == 0)
        {
            if (g_PointerLane > 0 && leftProfile.laneCount >= lanesRequired)
            {
                searchRanges.push_back(std::make_pair(1, leftProfile.laneCount));
                baseOffset = static_cast<double>(leftProfile.offsetx2) / 2;
                startFullyMatch = leftProfile.laneCount == lanesRequired && rightProfile.laneCount == 0;
            }
        }
        else
        {
            RoadRunner::SectionProfile prevProfile, nextProfile;
            if (g_PointerLane < 0)
            {
                prevProfile = g_roadProfile.ProfileAt(g_roadS - 0.01, -1);
                nextProfile = g_roadProfile.ProfileAt(g_roadS + 0.01, -1);
            }
            else
            {
                prevProfile = g_roadProfile.ProfileAt(g_roadS + 0.01, 1);
                nextProfile = g_roadProfile.ProfileAt(g_roadS - 0.01, 1);
            }

            if (prevProfile.offsetx2 == nextProfile.offsetx2)
            {
                // No offset change: can exit from right
                searchRanges.push_back(std::make_pair(nextProfile.laneCount, prevProfile.laneCount));
            }
            if (!g_roadProfile.HasSide(-startSide))
            {
                // Single-way: can exit from left
                auto delta = prevProfile.offsetx2 - nextProfile.offsetx2;
                if (delta % 2 == 0)
                {
                    startSplitOffset = delta / 2;
                    searchRanges.push_back(std::make_pair(1, std::min(
                        static_cast<RoadRunner::type_t>(startSplitOffset + 1),
                        prevProfile.laneCount)));
                }
                
            }
            baseOffset = static_cast<double>(prevProfile.offsetx2) / 2;
        }

        if (startFullyMatch)
        {
            auto snapPos = g_road->generated.get_xyz(g_roadS, 0, 0);
            if (g_roadS == 0)
            {
                rOffsetX2 = -leftProfile.offsetx2;
            }
            else
            {
                rOffsetX2 = rightProfile.offsetx2;
            }
            point.setX(snapPos[0]);
            point.setY(snapPos[1]);

            extendFromStart = g_road;
            if (g_roadS == 0) g_dir = -g_dir;
        }
        else
        {
            float bestError = 1.1e9;
            uint8_t bestSkip;
            QVector2D bestSnapPos;

            for (const auto& searchRange : searchRanges)
            {
                uint8_t searchLaneMin = searchRange.first;
                uint8_t searchLaneMax = searchRange.second;
                for (uint8_t searchInner = searchLaneMin, searchOuter = searchInner + lanesRequired - 1;
                    searchOuter <= searchLaneMax;
                    ++searchInner, ++searchOuter)
                {
                    double middleDist = static_cast<double>((int)searchInner + (int)searchOuter - 1) / 2;
                    double t = (baseOffset + startSide * middleDist) * RoadRunner::LaneWidth;
                    auto p = g_road->generated.get_xyz(g_roadS, t, 0);

                    QVector2D medianPos(p[0], p[1]);
                    float matchError = medianPos.distanceToPoint(QVector2D(point));
                    if (matchError < bestError)
                    {
                        bestSkip = searchInner - 1;
                        bestError = matchError;
                        bestSnapPos = medianPos;
                    }
                }
            }

            if (bestError < 1e9)
            {
                point.setX(bestSnapPos.x());
                point.setY(bestSnapPos.y());
                extendFromStart = g_road;
                startLanesSkip = bestSkip;
                lOffsetX2 = 0;
                rOffsetX2 = rLanes;
                if (startSide > 0) g_dir = -g_dir;
            }
        }
    }
    if (extendFromStart.expired())
    {
        return false;
    }
    else
    {
        extendFromStartS = g_roadS;
       
        startDir = std::make_unique<QVector2D>(g_dir);
        return true;
    }
}

bool LanesCreationSession::SnapLastPointToExisting(QPointF& point)
{
    auto g_road = g_PointerRoad.lock();
    if (g_road == nullptr)
    {
        return false;
    }

    const auto rightQuery = g_createRoadOption->RightResult();
    const auto leftQuery = g_createRoadOption->LeftResult();

    if (rLanes != rightQuery.laneCount || lLanes != leftQuery.laneCount)
    {
        if (extendFromStart.expired())
        {
            rLanes = rightQuery.laneCount;
            lLanes = leftQuery.laneCount;
        }
        else
        {
            // TODO: disable profile editing widget after first ctrl point placed
            spdlog::warn("Modification to config during ctrl points placement is ingnored!");
        }
    }

    if (rLanes == 0)
    {
        spdlog::warn("Right lanes must > 0");
        return false;
    }

    double g_roadS = GetAdjustedS();
    const auto& g_roadProfile = g_road->generated.rr_profile;
    const auto rightProfile = g_roadProfile.ProfileAt(g_roadS, -1);
    const auto leftProfile = g_roadProfile.ProfileAt(g_roadS, 1);
    endFullyMatch = false;
    endLanesSkip = 0;
    endSplitOffset = 0;

    if (extendFromStart.expired() && lLanes != 0)
    {
        if (g_roadS == 0)
        {
            rOffsetX2 = rightProfile.offsetx2;
            lOffsetX2 = leftProfile.offsetx2;
        }
        else
        {
            rOffsetX2 = -leftProfile.offsetx2;
            lOffsetX2 = -rightProfile.offsetx2;
        }
    }

    if (lLanes != 0)
    {
        // Bi-directional ramp
        if (g_roadS == 0 || g_roadS == g_road->Length())
        {
            int actualGap = std::abs(rightProfile.offsetx2 - leftProfile.offsetx2);
            int actualLeftLanes = g_roadS == 0 ? leftProfile.laneCount : rightProfile.laneCount;
            int actualRightLanes = g_roadS == 0 ? rightProfile.laneCount : leftProfile.laneCount;

            if (actualRightLanes >= rLanes
                && actualLeftLanes >= lLanes
                && actualGap == std::abs(rOffsetX2 - lOffsetX2))
            {
                joinAtEnd = g_road;
                // Last check of FullyMatch: Creating new road shouldn't affect original junction
                //      Form 2-road direct junction in this case
                endFullyMatch = actualRightLanes == rLanes && actualLeftLanes == lLanes
                    && (extendFromStart.expired() || rightProfile.offsetx2 == rOffsetX2 && leftProfile.offsetx2 == lOffsetX2);
                if (endFullyMatch)
                {
                    // Make sure ref line connects
                    auto snapPos = g_road->generated.get_xyz(g_roadS, 0, 0);
                    point.setX(snapPos[0]);
                    point.setY(snapPos[1]);
                }
                else
                {
                    RoadRunner::type_t refLineShift = 0;
                    if (!extendFromStart.expired())
                    {
                        if (g_roadS == g_road->Length())
                        {
                            refLineShift = leftProfile.offsetx2 + rOffsetX2;
                        }
                        else
                        {
                            refLineShift = rightProfile.offsetx2 - rOffsetX2;
                        }
                    }

                    auto snapPos = g_road->generated.get_xyz(g_roadS, RoadRunner::LaneWidth * refLineShift / 2, 0);
                    point.setX(snapPos[0]);
                    point.setY(snapPos[1]);
                }
            }
        }
    }
    else
    {
        // Single-directional ramp
        auto lanesRequired = rLanes;  // New road follows drawing direction
        std::vector<std::pair<uint8_t, uint8_t>> searchRanges;
        //uint8_t searchLaneMin = 1; // Inclusive
        //uint8_t searchLaneMax = 0; // Inclusive
        double baseOffset;
        endSide = g_PointerLane < 0 ? -1 : 1;
        if (g_roadS == 0)
        {
            if (g_PointerLane < 0 && rightProfile.laneCount >= lanesRequired)
            {
                searchRanges.push_back(std::make_pair(1, rightProfile.laneCount));
                //searchLaneMax = rightProfile.laneCount;
                baseOffset = static_cast<double>(rightProfile.offsetx2) / 2;
                endFullyMatch = rightProfile.laneCount == lanesRequired && leftProfile.laneCount == 0;
                rOffsetX2 = rightProfile.offsetx2;
                lOffsetX2 = leftProfile.offsetx2;
            }
        }
        else if (g_roadS == g_road->Length())
        {
            if (g_PointerLane > 0 && leftProfile.laneCount >= lanesRequired)
            {
                searchRanges.push_back(std::make_pair(1, leftProfile.laneCount));
                //searchLaneMax = leftProfile.laneCount;
                baseOffset = static_cast<double>(leftProfile.offsetx2) / 2;
                endFullyMatch = leftProfile.laneCount == lanesRequired && rightProfile.laneCount == 0;
                rOffsetX2 = -leftProfile.offsetx2;
                lOffsetX2 = -rightProfile.offsetx2;
            }
        }
        else
        {
            RoadRunner::SectionProfile prevProfile, nextProfile;
            if (g_PointerLane < 0)
            {
                prevProfile = g_roadProfile.ProfileAt(g_roadS - 0.01, -1);
                nextProfile = g_roadProfile.ProfileAt(g_roadS + 0.01, -1);
            }
            else
            {
                prevProfile = g_roadProfile.ProfileAt(g_roadS + 0.01, 1);
                nextProfile = g_roadProfile.ProfileAt(g_roadS - 0.01, 1);
            }

            if (prevProfile.offsetx2 == nextProfile.offsetx2)
            {
                // No offset change: can enter from right
                searchRanges.push_back(std::make_pair(prevProfile.laneCount, nextProfile.laneCount));
                //searchLaneMin = prevProfile.laneCount;
                //searchLaneMax = nextProfile.laneCount;
            }
            if (!g_roadProfile.HasSide(-startSide))
            {
                // Single-way: can enter from left
                auto delta = nextProfile.offsetx2 - prevProfile.offsetx2;
                if (delta % 2 == 0)
                {
                    endSplitOffset = delta / 2;
                    searchRanges.push_back(std::make_pair(1, std::min(
                        static_cast<RoadRunner::type_t>(endSplitOffset + 1),
                        nextProfile.laneCount)));
                }
            }

            baseOffset = static_cast<double>(nextProfile.offsetx2) / 2;
        }

        if (endFullyMatch)
        {
            // Make sure ref line connects
            auto snapPos = g_road->generated.get_xyz(g_roadS, 0 , 0);
            point.setX(snapPos[0]);
            point.setY(snapPos[1]);
            joinAtEnd = g_road;
        }
        else
        {
            float bestError = 1.1e9;
            int8_t bestSkip;
            QVector2D bestSnapPos;

            for (const auto& searchRange : searchRanges)
            {
                uint8_t searchLaneMin = searchRange.first;
                uint8_t searchLaneMax = searchRange.second;
                for (uint8_t searchInner = searchLaneMin, searchOuter = searchInner + lanesRequired - 1;
                    searchOuter <= searchLaneMax;
                    ++searchInner, ++searchOuter)
                {
                    double middleDist = static_cast<double>((int)searchInner + (int)searchOuter - 1) / 2;
                    double t = (baseOffset + endSide * middleDist) * RoadRunner::LaneWidth;
                    auto p = g_road->generated.get_xyz(g_roadS, t, 0);

                    QVector2D medianPos(p[0], p[1]);
                    float matchError = medianPos.distanceToPoint(QVector2D(point));
                    if (matchError < bestError)
                    {
                        bestSkip = searchInner - 1;
                        bestError = matchError;
                        bestSnapPos = medianPos;
                    }
                }
            }

            if (bestError < 1e9)
            {
                point.setX(bestSnapPos.x());
                point.setY(bestSnapPos.y());
                lOffsetX2 = 0;
                rOffsetX2 = rLanes;
                joinAtEnd = g_road;
                endLanesSkip = bestSkip;
            }
        }
    }

    if (joinAtEnd.expired())
    {
        return false;
    }
    else
    {
        joinAtEndS = g_roadS;
        return true;
    }
}

std::unique_ptr<odr::RoadGeometry> LanesCreationSession::CreateJoinAtEndGeo(bool forPreview) const
{
    auto joinPointDir = ForceDirection::None;
    if (lLanes == 0)
    {
        joinPointDir = g_PointerLane < 0 ? ForceDirection::Original : ForceDirection::Negate;
    }
    return RoadCreationSession::createJoinAtEndGeo(forPreview, joinPointDir);
}