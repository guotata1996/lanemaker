#include "road.h"
#include "junction.h"

#include <algorithm>
#include <vector>
#include <sstream>

#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"

#include "spdlog/spdlog.h"

namespace RoadRunner
{
    void Road::Generate(bool notifyJunctions)
    {
        profile.Apply(Length(), generated);
        generated.DeriveLaneBorders();
        IDGenerator::ForRoad()->NotifyChange(ID());

        if (notifyJunctions)
        {
            if (successorJunction != nullptr)
            {
                successorJunction->NotifyPotentialChange();
            }
            if (predecessorJunction != nullptr)
            {
                predecessorJunction->NotifyPotentialChange();
            }
        }
    }

    void Road::ReverseRefLine()
    {
        generated.ref_line.reverse();

        type_s length = from_odr_unit(Length());
        decltype(profile) newProfile(
            profile.RightEntrance().laneCount, -profile.RightEntrance().offsetx2,
            profile.LeftEntrance().laneCount,  -profile.LeftEntrance().offsetx2);

        for (const auto& lSectionInfo : profile.GetAllSections(length, 1))
        {
            type_s fwdStart = lSectionInfo.first.first;
            type_s fwdEnd = lSectionInfo.first.second;
            newProfile.OverwriteSection(-1, length - fwdStart, length - fwdEnd,
                lSectionInfo.second.laneCount, -lSectionInfo.second.offsetx2);
        }

        for (const auto& rSectionInfo : profile.GetAllSections(length, -1))
        {
            type_s fwdStart = rSectionInfo.first.first;
            type_s fwdEnd = rSectionInfo.first.second;
            newProfile.OverwriteSection(1, length - fwdStart, length - fwdEnd,
                rSectionInfo.second.laneCount, -rSectionInfo.second.offsetx2);
        }
        
        profile = newProfile;
        Generate(false);

        // Handle linkage
        generated.predecessor.type = odr::RoadLink::Type_None;
        generated.successor.type = odr::RoadLink::Type_None;

        std::swap(successorJunction, predecessorJunction);
        if (successorJunction != nullptr)
        {
            successorJunction->NotifyPotentialChange(ChangeInConnecting{shared_from_this(), ChangeInConnecting::Type_Reverse});
        }
        if (predecessorJunction != nullptr)
        {
            predecessorJunction->NotifyPotentialChange(ChangeInConnecting{ shared_from_this(), ChangeInConnecting::Type_Reverse });
        }
    }

    Road::~Road()
    {
        if (successorJunction != nullptr)
        {
            successorJunction->NotifyPotentialChange();
            successorJunction.reset();
        }
        if (predecessorJunction != nullptr)
        {
            predecessorJunction->NotifyPotentialChange();
            predecessorJunction.reset();
        }

        if (!ID().empty())
        {
            spdlog::trace("del road {}", ID());
            IDGenerator::ForRoad()->FreeID(ID());
        }
    }
} // namespace