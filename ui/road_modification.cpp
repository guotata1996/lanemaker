#include "road_drawing.h"
#include "junction.h"

#include <qevent.h>
#include "CreateRoadOptionWidget.h"

extern SectionProfileConfigWidget* g_createRoadOption;

RoadModificationSession::RoadModificationSession()
{}

bool RoadModificationSession::Update(const RoadRunner::MouseAction& evt)
{
    RoadDrawingSession::Update(evt);
    return RoadDestroySession::Update(evt);
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

    double sBegin = std::min(*s1, *s2);
    double sEnd = std::max(*s1, *s2);

    auto currBeginLeft = target->generated.rr_profile.ProfileAt(sBegin, 1);
    auto currBeginRight = target->generated.rr_profile.ProfileAt(sBegin, -1);
    auto currEndLeft = target->generated.rr_profile.ProfileAt(sEnd, 1);
    auto currEndRight = target->generated.rr_profile.ProfileAt(sEnd, -1);

    if (!(sBegin == 0 && sEnd == target->Length()) &&
        (target->generated.rr_profile.HasSide(1) != (g_createRoadOption->LeftResult().laneCount > 0) ||
        target->generated.rr_profile.HasSide(-1) != (g_createRoadOption->RightResult().laneCount > 0)))
    {
        if (sBegin != 0 && IsProfileChangePoint(target, sBegin)
            || sEnd != 0 && IsProfileChangePoint(target, sEnd))
        {
            spdlog::warn("Direct junction cannot be created next to changing profile.");
            return true;
        }

        // Form direct junction
        std::shared_ptr<RoadRunner::Road> before, toModify, after;
        if (sEnd != target->Length())
        {
            after = RoadRunner::Road::SplitRoad(target, sEnd);
            World::Instance()->allRoads.insert(after);
            toModify = target;
        }

        if (sBegin != 0)
        {
            toModify = RoadRunner::Road::SplitRoad(target, sBegin);
            World::Instance()->allRoads.insert(toModify);
            before = target;
        }

        toModify->ModifyProfile(0, toModify->Length(), g_createRoadOption->LeftResult(), g_createRoadOption->RightResult());
        if (before != nullptr)
        {
            if (currBeginLeft.laneCount != 0 && g_createRoadOption->LeftResult().laneCount != 0
                && currBeginLeft.offsetx2 != g_createRoadOption->LeftResult().offsetx2
                ||
                currBeginRight.laneCount != 0 && g_createRoadOption->RightResult().laneCount != 0
                && currBeginRight.offsetx2 != g_createRoadOption->RightResult().offsetx2)
            {
                spdlog::warn("Offset must remain the same while creating direct junction.");
                return false;
            }

            auto toModifyInfo = RoadRunner::ConnectionInfo(toModify, odr::RoadLink::ContactPoint_Start);
            auto beforeInfo = RoadRunner::ConnectionInfo(before, odr::RoadLink::ContactPoint_End);

            if (currBeginLeft.laneCount <= g_createRoadOption->LeftResult().laneCount
                && currBeginRight.laneCount <= g_createRoadOption->RightResult().laneCount)
            {
                auto junc = std::make_shared< RoadRunner::DirectJunction>(toModifyInfo);
                junc->Attach(beforeInfo);
            }
            else if (currBeginLeft.laneCount >= g_createRoadOption->LeftResult().laneCount
                && currBeginRight.laneCount >= g_createRoadOption->RightResult().laneCount)
            {
                auto junc = std::make_shared< RoadRunner::DirectJunction>(beforeInfo);
                junc->Attach(toModifyInfo);
            }
            else
            {
                spdlog::warn("Can't find interface provider for direct junction.");
                return false;
            }
        }
        if (after != nullptr)
        {
            if (currEndLeft.laneCount != 0 && g_createRoadOption->LeftResult().laneCount != 0
                && currEndLeft.offsetx2 != g_createRoadOption->LeftResult().offsetx2
                ||
                currEndRight.laneCount != 0 && g_createRoadOption->RightResult().laneCount != 0
                && currEndRight.offsetx2 != g_createRoadOption->RightResult().offsetx2)
            {
                spdlog::warn("Offset must remain the same while creating direct junction!");
                return false;
            }

            auto toModifyInfo = RoadRunner::ConnectionInfo(toModify, odr::RoadLink::ContactPoint_End);
            auto afterInfo = RoadRunner::ConnectionInfo(after, odr::RoadLink::ContactPoint_Start);

            if (currEndLeft.laneCount <= g_createRoadOption->LeftResult().laneCount
                && currEndRight.laneCount <= g_createRoadOption->RightResult().laneCount)
            {
                auto junc = std::make_shared< RoadRunner::DirectJunction>(toModifyInfo);
                junc->Attach(afterInfo);
            }
            else if (currEndLeft.laneCount >= g_createRoadOption->LeftResult().laneCount
                && currEndRight.laneCount >= g_createRoadOption->RightResult().laneCount)
            {
                auto junc = std::make_shared< RoadRunner::DirectJunction>(afterInfo);
                junc->Attach(toModifyInfo);
            }
            else
            {
                spdlog::warn("Can't find interface provider for direct junction!");
                return false;
            }
        }

        UpdateEndMarkings();
        return true;
    }

    bool success = target->ModifyProfile(sBegin, sEnd, g_createRoadOption->LeftResult(), g_createRoadOption->RightResult());
    if (success)
    {
        UpdateEndMarkings();
    }
    return success;
}