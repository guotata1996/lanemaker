#include "road.h"
#include "junction.h"

#include <algorithm>
#include <vector>
#include <sstream>

#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"

#include "spdlog/spdlog.h"

#include "RoadGraphics.h"

namespace RoadRunner
{
    Road::Road(const RoadProfile& p, std::shared_ptr<odr::RoadGeometry> l) :
        generated(IDGenerator::ForRoad()->GenerateID(this), 0, "-1"),
        profile(p)
    {
        generated.ref_line.length = l->length;
        generated.ref_line.s0_to_geometry.emplace(0, l->clone());
        Generate();
        //GenerateAllSectionGraphics();
    }

    Road::Road(const RoadProfile& p, odr::RefLine& l) :
        generated(IDGenerator::ForRoad()->GenerateID(this), 0, "-1"),
        profile(p)
    {
        generated.ref_line = std::move(l);
        Generate();
    }

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
            s_to_section_graphics.clear(); // ROADRUNNERTODO: don't need to clear everything
        }
    }

    void Road::GenerateAllSectionGraphics()
    {
        s_to_section_graphics.clear();

        for (auto it = generated.s_to_lanesection.begin();
            it != generated.s_to_lanesection.end(); ++it)
        {
            auto nextIt = it;
            nextIt++;
            double startS = it->first;
            double endS = nextIt == generated.s_to_lanesection.end() ? generated.length : nextIt->first;
            
            if (endS - startS < 0.1f)
            {
                continue;
            }

            odr::LaneSection& section = it->second;
            int nDivision = std::floor((endS - startS) / GraphicsDivision);
            nDivision = std::max(1, nDivision);
            double segmentLength = (endS - startS) / nDivision;
            for (int segmentIndex = 0; segmentIndex != nDivision; ++segmentIndex)
            {
                double segStartS = startS + segmentLength * segmentIndex;
                double segEndS = segmentIndex == nDivision - 1 ? endS : segStartS + segmentLength;
                auto sectionGraphics = std::make_unique<RoadGraphics>(shared_from_this());
                sectionGraphics->Update(it->second, segStartS, segEndS);
                s_to_section_graphics.emplace(segStartS, std::move(sectionGraphics));
            }
        }
    }
} // namespace