#include "road_drawing.h"
#include "junction.h"

#include "LaneConfigWidget.h"

RoadModificationSession::RoadModificationSession()
{}

bool RoadModificationSession::Update(const LM::MouseAction& evt)
{
    RoadDrawingSession::Update(evt);
    return RoadDestroySession::Update(evt);
}

bool RoadModificationSession::Complete()
{
    if (targetRoad.expired() || s2 == nullptr) return true;

    auto target = targetRoad.lock();
    if (std::min(*s1, *s2) == 0 &&
        dynamic_cast<LM::DirectJunction*>(target->predecessorJunction.get()) != nullptr
        ||
        std::max(*s1, *s2) == target->Length() &&
        dynamic_cast<LM::DirectJunction*>(target->successorJunction.get()) != nullptr)
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
        (target->generated.rr_profile.HasSide(1) != (g_laneConfig->LeftResult().laneCount > 0) ||
        target->generated.rr_profile.HasSide(-1) != (g_laneConfig->RightResult().laneCount > 0)))
    {
        if (sBegin != 0 && IsProfileChangePoint(target, sBegin)
            || sEnd != 0 && IsProfileChangePoint(target, sEnd))
        {
            spdlog::warn("Direct junction cannot be created next to changing profile.");
            return true;
        }

        // Form direct junction
        std::shared_ptr<LM::Road> before, toModify, after;
        if (sEnd != target->Length())
        {
            after = LM::Road::SplitRoad(target, sEnd);
            World::Instance()->allRoads.insert(after);
            toModify = target;
        }

        if (sBegin != 0)
        {
            toModify = LM::Road::SplitRoad(target, sBegin);
            World::Instance()->allRoads.insert(toModify);
            before = target;
        }

        toModify->ModifyProfile(0, toModify->Length(), g_laneConfig->LeftResult(), g_laneConfig->RightResult());
        if (before != nullptr)
        {
            if (currBeginLeft.laneCount != 0 && g_laneConfig->LeftResult().laneCount != 0
                && currBeginLeft.offsetx2 != g_laneConfig->LeftResult().offsetx2
                ||
                currBeginRight.laneCount != 0 && g_laneConfig->RightResult().laneCount != 0
                && currBeginRight.offsetx2 != g_laneConfig->RightResult().offsetx2)
            {
                spdlog::warn("Offset must remain the same while creating direct junction.");
                return false;
            }

            auto toModifyInfo = LM::ConnectionInfo(toModify, odr::RoadLink::ContactPoint_Start);
            auto beforeInfo = LM::ConnectionInfo(before, odr::RoadLink::ContactPoint_End);

            if (currBeginLeft.laneCount <= g_laneConfig->LeftResult().laneCount
                && currBeginRight.laneCount <= g_laneConfig->RightResult().laneCount)
            {
                auto junc = std::make_shared< LM::DirectJunction>(toModifyInfo);
                junc->Attach(beforeInfo);
            }
            else if (currBeginLeft.laneCount >= g_laneConfig->LeftResult().laneCount
                && currBeginRight.laneCount >= g_laneConfig->RightResult().laneCount)
            {
                auto junc = std::make_shared< LM::DirectJunction>(beforeInfo);
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
            if (currEndLeft.laneCount != 0 && g_laneConfig->LeftResult().laneCount != 0
                && currEndLeft.offsetx2 != g_laneConfig->LeftResult().offsetx2
                ||
                currEndRight.laneCount != 0 && g_laneConfig->RightResult().laneCount != 0
                && currEndRight.offsetx2 != g_laneConfig->RightResult().offsetx2)
            {
                spdlog::warn("Offset must remain the same while creating direct junction!");
                return false;
            }

            auto toModifyInfo = LM::ConnectionInfo(toModify, odr::RoadLink::ContactPoint_End);
            auto afterInfo = LM::ConnectionInfo(after, odr::RoadLink::ContactPoint_Start);

            if (currEndLeft.laneCount <= g_laneConfig->LeftResult().laneCount
                && currEndRight.laneCount <= g_laneConfig->RightResult().laneCount)
            {
                auto junc = std::make_shared< LM::DirectJunction>(toModifyInfo);
                junc->Attach(afterInfo);
            }
            else if (currEndLeft.laneCount >= g_laneConfig->LeftResult().laneCount
                && currEndRight.laneCount >= g_laneConfig->RightResult().laneCount)
            {
                auto junc = std::make_shared< LM::DirectJunction>(afterInfo);
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

    bool success = target->ModifyProfile(sBegin, sEnd, g_laneConfig->LeftResult(), g_laneConfig->RightResult());
    if (success)
    {
        UpdateEndMarkings();
    }
    return success;
}