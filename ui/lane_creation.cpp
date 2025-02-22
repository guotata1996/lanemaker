#include "road_drawing.h"

#include <qvector2d.h>
#include <qevent.h>

#include "CreateRoadOptionWidget.h"
#include "junction.h"
#include "constants.h"
#include "road_drawing.h"
#include "road_overlaps.h"
#include "map_view_gl.h"

extern SectionProfileConfigWidget* g_createRoadOption;

LanesCreationSession::LanesCreationSession() :
    lLanes(0), rLanes(0), rOffsetX2(0), lOffsetX2(0)
{

}

bool LanesCreationSession::Complete()
{
    if (extendFromStart.expired() && joinAtEnd.expired())
    {
        // Standalone road
        return RoadCreationSession::Complete();
    }

    auto refLine = ResultRefLine();
    if (refLine.length == 0)
    {
        spdlog::warn("Too few control points / curve too sharp");
        return true;
    }

    if (refLine.length > LM::SingleDrawMaxLength)
    {
        spdlog::warn("Invalid shape or Road to create is too long");
        return true;
    }

    if (!extendFromStart.expired() && !joinAtEnd.expired() && extendFromStart.lock() == joinAtEnd.lock() &&
        !(extendFromStartS == 0 && joinAtEndS == 0 ||
            extendFromStartS == extendFromStart.lock()->Length() && joinAtEndS == joinAtEnd.lock()->Length()))
    {
        spdlog::warn("Cannot operate on the same road twice at a time.");
        return true;
    }
    LM::LaneProfile config;
    config = LM::LaneProfile(lLanes, lOffsetX2, rLanes, rOffsetX2);

    auto newRoad = std::make_shared<LM::Road>(config, refLine);
    newRoad->GenerateAllSectionGraphics();

    bool standaloneRoad = true;
    double newPartBegin = 0, newPartEnd = newRoad->Length();
    if (!extendFromStart.expired())
    {
        auto toExtend = extendFromStart.lock();
        LM::ConnectionInfo linkedInfo(newRoad, odr::RoadLink::ContactPoint_Start, startLanesSkip);
        if (extendFromStartS == 0 || extendFromStartS == toExtend->Length())
        {
            std::shared_ptr<LM::AbstractJunction> directJunction;
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
                if (directJunction->Attach(linkedInfo) != LM::JunctionError::Junction_NoError)
                {
                    return false;
                }
            }
            else if (startFullyMatch)
            {
                standaloneRoad = false;
                auto joinPointElevation = toExtend->RefLine().elevation_profile.get(extendFromStartS);
                LM::CubicSplineGenerator::OverwriteSection(
                    newRoad->RefLine().elevation_profile, newRoad->Length(), 0, 0, joinPointElevation);
                newPartBegin += toExtend->Length();
                newPartEnd += toExtend->Length();
                int joinResult = LM::Road::JoinRoads(toExtend,
                    extendFromStartS == 0 ? odr::RoadLink::ContactPoint_Start : odr::RoadLink::ContactPoint_End,
                    newRoad, odr::RoadLink::ContactPoint_Start);
                if (joinResult != 0)
                {
                    spdlog::error("LanesCreationSession:: Extend error {}", joinResult);
                    return false;
                }

                newRoad = toExtend;
            }
            else
            {
                if (extendFromStartS == 0)
                {
                    directJunction = std::make_shared<LM::DirectJunction>(
                        LM::ConnectionInfo(toExtend, odr::RoadLink::ContactPoint_Start));
                }
                else
                {
                    directJunction = std::make_shared<LM::DirectJunction>(
                        LM::ConnectionInfo(toExtend, odr::RoadLink::ContactPoint_End));
                }
                directJunction->Attach(linkedInfo);
            }
        }
        else
        {
            auto secondHalf = LM::Road::SplitRoad(toExtend, extendFromStartS);
            world->allRoads.insert(secondHalf);

            std::shared_ptr<LM::AbstractJunction> directJunction;
            if (startSide < 0)
            {
                directJunction = std::make_shared< LM::DirectJunction>(
                    LM::ConnectionInfo(toExtend, odr::RoadLink::ContactPoint_End));
                directJunction->Attach(LM::ConnectionInfo(secondHalf, odr::RoadLink::ContactPoint_Start, startSplitOffset));
            }
            else
            {
                directJunction = std::make_shared< LM::DirectJunction>(
                    LM::ConnectionInfo(secondHalf, odr::RoadLink::ContactPoint_Start));
                directJunction->Attach(LM::ConnectionInfo(toExtend, odr::RoadLink::ContactPoint_End, startSplitOffset));
            }
            directJunction->Attach(linkedInfo);
        }
    }
    
    if (!joinAtEnd.expired())
    {
        auto toJoin = joinAtEnd.lock();
        LM::ConnectionInfo linkedInfo(newRoad, odr::RoadLink::ContactPoint_End, endLanesSkip);
        if (joinAtEndS == 0 || joinAtEndS == toJoin->Length())
        {
            std::shared_ptr<LM::AbstractJunction> directJunction;
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
                if (directJunction->Attach(linkedInfo) != LM::JunctionError::Junction_NoError)
                {
                    return false;
                }
            }
            else if (endFullyMatch)
            {
                if (toJoin == newRoad)
                {
                    spdlog::warn("Self-loop is not supported! Revoking...");
                    return false;
                }
                standaloneRoad = false;
                world->allRoads.erase(toJoin);
                auto joinPointElevation = toJoin->RefLine().elevation_profile.get(joinAtEndS);
                LM::CubicSplineGenerator::OverwriteSection(
                    newRoad->RefLine().elevation_profile, newRoad->Length(), 
                    newRoad->Length(), newRoad->Length(), joinPointElevation);

                int joinResult = LM::Road::JoinRoads(newRoad, odr::RoadLink::ContactPoint_End,
                    toJoin, joinAtEndS == 0 ? odr::RoadLink::ContactPoint_Start : odr::RoadLink::ContactPoint_End);
                if (joinResult != 0)
                {
                    spdlog::error("LanesCreationSession:: Join error {}", joinResult);
                    return false;
                }
                world->allRoads.insert(newRoad);
            }
            else
            {
                if (joinAtEndS == 0)
                {
                    directJunction = std::make_shared<LM::DirectJunction>(
                        LM::ConnectionInfo(toJoin, odr::RoadLink::ContactPoint_Start));
                }
                else
                {
                    directJunction = std::make_shared<LM::DirectJunction>(
                        LM::ConnectionInfo(toJoin, odr::RoadLink::ContactPoint_End));
                }
                directJunction->Attach(linkedInfo);
            }
        }
        else
        {
            auto secondHalf = LM::Road::SplitRoad(toJoin, joinAtEndS);
            world->allRoads.insert(secondHalf);

            std::shared_ptr<LM::AbstractJunction> directJunction;
            if (endSide < 0)
            {
                directJunction = std::make_shared< LM::DirectJunction>(
                    LM::ConnectionInfo(secondHalf, odr::RoadLink::ContactPoint_Start));
                directJunction->Attach(LM::ConnectionInfo(toJoin, odr::RoadLink::ContactPoint_End, endSplitOffset));
            }
            else
            {
                directJunction = std::make_shared<LM::DirectJunction>(
                    LM::ConnectionInfo(toJoin, odr::RoadLink::ContactPoint_End));
                directJunction->Attach(LM::ConnectionInfo(secondHalf, odr::RoadLink::ContactPoint_Start, endSplitOffset));
            }
            directJunction->Attach(linkedInfo);
        }
    }
    
    if (standaloneRoad)
    {
        world->allRoads.insert(newRoad);
    }

    bool success = CreateJunctionAtZOverlap(std::move(newRoad), newPartBegin, newPartEnd);
    if (success)
    {
        UpdateEndMarkings();
    }
    return success;
}

// lane editing cannot happen to 
// 1) connecting roads
// 2) at normal junction interface
// 3) Where lane offset changes (on either side)
bool LanesCreationSession::ValidateSnap() const
{
    auto g_road = GetPointerRoad();
    if (g_road == nullptr)
    {
        return false;
    }
    double g_roadS = GetAdjustedS();
    if (g_road->generated.junction != "-1")
    {
        return false;
    }
    if (g_roadS == 0 && g_road->predecessorJunction != nullptr &&
        dynamic_cast<LM::Junction*>(g_road->predecessorJunction.get()) != nullptr
        || g_roadS == g_road->Length() && g_road->successorJunction != nullptr &&
        dynamic_cast<LM::Junction*>(g_road->successorJunction.get()) != nullptr)
    {
        return false;
    }
    return true;
}

RoadDrawingSession::SnapResult LanesCreationSession::SnapFirstPointToExisting(odr::Vec2D& point)
{
    if (!ValidateSnap())
    {
        return RoadDrawingSession::Snap_Nothing;
    }
    auto g_road = GetPointerRoad();
    
    rLanes = g_createRoadOption->RightResult().laneCount;
    lLanes = g_createRoadOption->LeftResult().laneCount;

    if (rLanes == 0)
    {
        spdlog::warn("Right lanes must > 0");
        return RoadDrawingSession::Snap_Nothing;
    }
        
    bool snappedToSegBoundary = false;
    double g_roadS = GetAdjustedS(&snappedToSegBoundary);

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
                point = g_road->generated.get_xy(g_roadS);

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
        startSide = LM::g_PointerLane < 0 ? -1 : 1;
        if (g_roadS == g_road->Length())
        {
            if (LM::g_PointerLane < 0 && rightProfile.laneCount >= lanesRequired)
            {
                searchRanges.push_back(std::make_pair(1, rightProfile.laneCount));
                baseOffset = static_cast<double>(rightProfile.offsetx2) / 2;
                startFullyMatch = rightProfile.laneCount == lanesRequired && leftProfile.laneCount == 0;
            }
        }
        else if (g_roadS == 0)
        {
            if (LM::g_PointerLane > 0 && leftProfile.laneCount >= lanesRequired)
            {
                searchRanges.push_back(std::make_pair(1, leftProfile.laneCount));
                baseOffset = static_cast<double>(leftProfile.offsetx2) / 2;
                startFullyMatch = leftProfile.laneCount == lanesRequired && rightProfile.laneCount == 0;
            }
        }
        else
        {
            LM::LanePlan prevProfile, nextProfile;
            if (LM::g_PointerLane < 0)
            {
                prevProfile = g_roadProfile.ProfileAt(g_roadS - 0.01, -1);
                nextProfile = g_roadProfile.ProfileAt(g_roadS + 0.01, -1);
            }
            else
            {
                prevProfile = g_roadProfile.ProfileAt(g_roadS + 0.01, 1);
                nextProfile = g_roadProfile.ProfileAt(g_roadS - 0.01, 1);
            }

            if (!IsProfileChangePoint(g_road, g_roadS))
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
                        static_cast<LM::type_t>(startSplitOffset + 1),
                        prevProfile.laneCount)));
                }
                
            }
            baseOffset = static_cast<double>(prevProfile.offsetx2) / 2;
        }

        if (startFullyMatch)
        {
            point = g_road->generated.get_xy(g_roadS);
            if (g_roadS == 0)
            {
                rOffsetX2 = -leftProfile.offsetx2;
            }
            else
            {
                rOffsetX2 = rightProfile.offsetx2;
            }

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
                    double t = (baseOffset + startSide * middleDist) * LM::LaneWidth;
                    auto p = g_road->generated.get_xyz(g_roadS, t, 0);

                    QVector2D medianPos(p[0], p[1]);
                    float matchError = medianPos.distanceToPoint(QVector2D(point[0], point[1]));
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
                point[0] = bestSnapPos.x();
                point[1] = bestSnapPos.y();
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
        return RoadDrawingSession::Snap_Nothing;
    }
    else
    {
        extendFromStartS = g_roadS;
       
        return snappedToSegBoundary ? RoadDrawingSession::Snap_Point : RoadDrawingSession::Snap_Line;
    }
}

RoadDrawingSession::SnapResult LanesCreationSession::SnapLastPointToExisting(odr::Vec2D& point)
{
    if (!ValidateSnap())
    {
        return RoadDrawingSession::Snap_Nothing;
    }

    auto g_road = GetPointerRoad();

    if (rLanes != g_createRoadOption->RightResult().laneCount ||
        lLanes != g_createRoadOption->LeftResult().laneCount)
    {
        if (extendFromStart.expired())
        {
            rLanes = g_createRoadOption->RightResult().laneCount;
            lLanes = g_createRoadOption->LeftResult().laneCount;
        }
        else
        {
            spdlog::warn("Modification to config during ctrl points placement is ingnored!");
        }
    }

    if (rLanes == 0)
    {
        spdlog::warn("Right lanes must > 0");
        return RoadDrawingSession::Snap_Nothing;
    }

    bool snappedToSegBoundary = false;
    const double g_roadS = GetAdjustedS(&snappedToSegBoundary);

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
                    point = g_road->generated.get_xy(g_roadS);
                }
                else
                {
                    LM::type_t refLineShift = 0;
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

                    point = g_road->generated.get_xy(g_roadS, LM::LaneWidth * refLineShift / 2);
                }
            }
        }
    }
    else
    {
        // Single-directional ramp
        auto lanesRequired = rLanes;  // New road follows drawing direction
        std::vector<std::pair<uint8_t, uint8_t>> searchRanges;
        double baseOffset;
        endSide = LM::g_PointerLane < 0 ? -1 : 1;
        if (g_roadS == 0)
        {
            if (LM::g_PointerLane < 0 && rightProfile.laneCount >= lanesRequired)
            {
                searchRanges.push_back(std::make_pair(1, rightProfile.laneCount));
                baseOffset = static_cast<double>(rightProfile.offsetx2) / 2;
                endFullyMatch = rightProfile.laneCount == lanesRequired && leftProfile.laneCount == 0;
                rOffsetX2 = rightProfile.offsetx2;
                lOffsetX2 = leftProfile.offsetx2;
            }
        }
        else if (g_roadS == g_road->Length())
        {
            if (LM::g_PointerLane > 0 && leftProfile.laneCount >= lanesRequired)
            {
                searchRanges.push_back(std::make_pair(1, leftProfile.laneCount));
                baseOffset = static_cast<double>(leftProfile.offsetx2) / 2;
                endFullyMatch = leftProfile.laneCount == lanesRequired && rightProfile.laneCount == 0;
                rOffsetX2 = -leftProfile.offsetx2;
                lOffsetX2 = -rightProfile.offsetx2;
            }
        }
        else
        {
            LM::LanePlan prevProfile, nextProfile;
            if (LM::g_PointerLane < 0)
            {
                prevProfile = g_roadProfile.ProfileAt(g_roadS - 0.01, -1);
                nextProfile = g_roadProfile.ProfileAt(g_roadS + 0.01, -1);
            }
            else
            {
                prevProfile = g_roadProfile.ProfileAt(g_roadS + 0.01, 1);
                nextProfile = g_roadProfile.ProfileAt(g_roadS - 0.01, 1);
            }

            if (!IsProfileChangePoint(g_road, g_roadS))
            {
                // No offset change: can enter from right
                searchRanges.push_back(std::make_pair(prevProfile.laneCount, nextProfile.laneCount));
            }
            if (!g_roadProfile.HasSide(-endSide))
            {
                // Single-way: can enter from left
                auto delta = nextProfile.offsetx2 - prevProfile.offsetx2;
                if (delta % 2 == 0)
                {
                    endSplitOffset = delta / 2;
                    searchRanges.push_back(std::make_pair(1, std::min(
                        static_cast<LM::type_t>(endSplitOffset + 1),
                        nextProfile.laneCount)));
                }
            }

            baseOffset = static_cast<double>(nextProfile.offsetx2) / 2;
        }

        if (endFullyMatch)
        {
            // Make sure ref line connects
            point = g_road->generated.get_xy(g_roadS);
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
                    double t = (baseOffset + endSide * middleDist) * LM::LaneWidth;
                    auto p = g_road->generated.get_xy(g_roadS, t);
                    QVector2D medianPos(p[0], p[1]);
                    float matchError = medianPos.distanceToPoint(QVector2D(point[0], point[1]));
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
                point[0] = bestSnapPos.x();
                point[1] = bestSnapPos.y();
                joinAtEnd = g_road;
                endLanesSkip = bestSkip;
            }
        }

        lOffsetX2 = 0;
        rOffsetX2 = rLanes;
    }

    if (joinAtEnd.expired())
    {
        return RoadDrawingSession::Snap_Nothing;
    }
    else
    {
        joinAtEndS = g_roadS;
        return snappedToSegBoundary ? RoadDrawingSession::Snap_Point : RoadDrawingSession::Snap_Line;
    }
}

odr::Vec2D LanesCreationSession::ExtendFromDir() const
{
    if (lLanes != 0)
    {
        return RoadCreationSession::ExtendFromDir();
    }
    auto grad = odr::normalize(extendFromStart.lock()->RefLine().get_grad_xy(extendFromStartS));
    return startSide > 0 ? odr::negate(grad) : grad;
}

odr::Vec2D LanesCreationSession::JoinAtEndDir() const
{
    if (lLanes != 0)
    {
        return RoadCreationSession::JoinAtEndDir();
    }
    auto grad = odr::normalize(joinAtEnd.lock()->RefLine().get_grad_xy(joinAtEndS));
    return endSide > 0 ? odr::negate(grad) : grad;
}

LM::type_t LanesCreationSession::PreviewRightOffsetX2() const
{
    return extendFromStart.expired() ? RoadCreationSession::PreviewRightOffsetX2() : rOffsetX2;
}

LM::type_t LanesCreationSession::PreviewLeftOffsetX2() const
{
    return extendFromStart.expired() ? RoadCreationSession::PreviewLeftOffsetX2() :
        (lLanes == 0 ? -rOffsetX2 : lOffsetX2);
}
