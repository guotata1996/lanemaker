#include "road.h"
#include "curve_fitting.h"
#include "junction.h"
#ifndef G_TEST
#include "road_graphics.h"
#endif

#include <math.h>

namespace RoadRunner
{
    enum RoadJoinError
    {
        RoadJoin_Success,
        RoadJoin_InputIsIdentical,
        RoadJoin_DirNoOutlet,
        RoadJoin_ConnectionInvalidShape
    };

    int Road::JoinRoads(std::shared_ptr<Road>& road1, odr::RoadLink::ContactPoint c1,
        std::shared_ptr<Road>& road2, odr::RoadLink::ContactPoint c2)
    {
        if (road1 == road2 && c1 == c2)
        {
            return RoadJoin_InputIsIdentical;
        }

        if (c1 == odr::RoadLink::ContactPoint_Start)
        {
            road1->ReverseRefLine();
        }
        if (c2 == odr::RoadLink::ContactPoint_End)
        {
            road2->ReverseRefLine();
        }

        if (road1->profile.RightExit().laneCount > 0 != road2->profile.RightEntrance().laneCount > 0
            || road2->profile.LeftExit().laneCount > 0 != road1->profile.LeftEntrance().laneCount > 0)
        {
            return RoadJoin_DirNoOutlet;
        }

        auto p1 = road1->RefLine().get_xy(road1->Length());
        auto f1 = road1->RefLine().get_grad_xy(road1->Length());
        auto h1 = std::atan2(f1[1], f1[0]);
        auto p2 = road2->RefLine().get_xy(0);
        auto f2 = road2->RefLine().get_grad_xy(0);
        auto h2 = std::atan2(f2[1], f2[0]);
        bool hdgClose = std::abs(h1 - h2) < 1e-2 || std::abs(std::abs(h1 - h2) - M_PI * 2) < 1e-2;
        type_s linkBase = from_odr_unit(road1->Length());
        type_s road2Base;
        double linkBaseD = road1->Length();
        double road2BaseD;

        if (odr::euclDistance(p1, p2) > 1e-2 || !hdgClose)
        {
            auto connectingRef = ConnectLines(p1, odr::normalize(f1), p2, odr::normalize(f2));
            if (connectingRef == nullptr)
            {
                return RoadJoin_ConnectionInvalidShape;
            }
            road2BaseD = road1->Length() + connectingRef->length;
            road2Base = from_odr_unit(road2BaseD);
            // Connect ref lines
            connectingRef->s0 = road1->Length();
            road1->generated.ref_line.s0_to_geometry.emplace(road1->Length(), connectingRef->clone());
            double baseLength = road1->Length() + connectingRef->length;
            for (const auto& s2geo : road2->RefLine().s0_to_geometry)
            {
                auto clonedLine = s2geo.second->clone();
                clonedLine->s0 = baseLength + s2geo.first;
                road1->generated.ref_line.s0_to_geometry.emplace(
                    baseLength + s2geo.first,
                    std::move(clonedLine));
            }
            road1->generated.ref_line.length = road1->Length() + connectingRef->length + road2->Length();
        }
        else
        {
            road2Base = linkBase;
            road2BaseD = linkBaseD;
            // Ref line already meet
            for (const auto& s2geo : road2->RefLine().s0_to_geometry)
            {
                auto clonedLine = s2geo.second->clone();
                clonedLine->s0 = road1->Length() + s2geo.first;
                road1->generated.ref_line.s0_to_geometry.emplace(
                    road1->Length() + s2geo.first,
                    std::move(clonedLine));
            }
            road1->generated.ref_line.length = road1->Length() + road2->Length();
        }
#ifndef G_TEST
        road1->SnapToSegmentBoundary(linkBase);
        road1->SnapToSegmentBoundary(road2Base);
#endif
        // Join right profile
        if (road1->profile.HasSide(-1))
        {
            if (road2Base != linkBase)
            {
                road1->profile.OverwriteSection(-1,
                    linkBase, road2Base,
                    road1->profile.RightExit().laneCount, road1->profile.RightExit().offsetx2);
            }

            for (const auto& road2section : road2->profile.GetAllSections(from_odr_unit(road2->Length()), -1))
            {
                type_s newStart = road2section.first.first + road2Base;
                type_s newEnd = road2section.first.second + road2Base;
#ifndef G_TEST
                road1->SnapToSegmentBoundary(newStart);
                road1->SnapToSegmentBoundary(newEnd);
#endif
                road1->profile.OverwriteSection(-1, newStart, newEnd,
                    road2section.second.laneCount, road2section.second.offsetx2);
            }
        }
        // Join left profile
        if (road1->profile.HasSide(1))
        {
            if (road2Base != linkBase)
            {
                road1->profile.OverwriteSection(1,
                    road2Base, linkBase,
                    road1->profile.LeftEntrance().laneCount, road1->profile.LeftEntrance().offsetx2);
            }

            for (const auto& road2section : road2->profile.GetAllSections(from_odr_unit(road2->Length()), 1))
            {
                type_s newStart = road2section.first.first + road2Base;
                type_s newEnd = road2section.first.second + road2Base;
#ifndef G_TEST
                road1->SnapToSegmentBoundary(newStart);
                road1->SnapToSegmentBoundary(newEnd);
#endif
                road1->profile.OverwriteSection(1, newStart, newEnd,
                    road2section.second.laneCount, road2section.second.offsetx2);
            }
        }

        road1->Generate();
#ifndef G_TEST
        for (auto& s_graphics : road2->s_to_section_graphics)
        {
            auto graphics = std::move(s_graphics.second);
            graphics->road = road1;
            graphics->sBegin += road2BaseD;
            graphics->sEnd += road2BaseD;
            road1->s_to_section_graphics.emplace(road2BaseD + s_graphics.first, std::move(graphics));
        }
#endif

        if (road2->successorJunction != nullptr)
        {
            road2->successorJunction->NotifyPotentialChange(ChangeInConnecting
                { road2, RoadRunner::ChangeInConnecting::Type_DetachAtEnd_Temp });
            road2->successorJunction->Attach(RoadRunner::ConnectionInfo{ road1, odr::RoadLink::ContactPoint_End });
        }
#ifndef G_TEST
        // Update transition part, up to max transition length
        road1->GenerateOrUpdateSectionGraphicsBetween(
            std::max(0.0, linkBaseD - 3 * MaxTransition),
            std::min(road2BaseD + 3 * MaxTransition, road1->Length()));
#endif
        road2.reset();
        return RoadJoin_Success;
    }

    std::shared_ptr<Road> Road::SplitRoad(std::shared_ptr<Road>& roadAsPrev, double s)
    {
        type_s oldLength = from_odr_unit(roadAsPrev->Length());
        type_s splitPoint = from_odr_unit(s);
        RoadProfile& oldProfile = roadAsPrev->profile;
        RoadProfile profile1(oldProfile.LeftExit().laneCount, oldProfile.LeftExit().offsetx2,
            oldProfile.RightEntrance().laneCount, oldProfile.RightEntrance().offsetx2);
        RoadProfile profile2(oldProfile.LeftEntrance().laneCount, oldProfile.LeftEntrance().offsetx2,
            oldProfile.RightExit().laneCount, oldProfile.RightExit().offsetx2);
        
        for (auto section : oldProfile.GetAllSections(oldLength, -1))
        {
            type_s sectionBegin = section.first.first;
            type_s sectionEnd = section.first.second;
            if (sectionEnd <= splitPoint)
            {
                profile1.OverwriteSection(-1, sectionBegin, sectionEnd, section.second.laneCount, section.second.offsetx2);
            }
            else if (sectionBegin < splitPoint && splitPoint < sectionEnd)
            {
                profile1.OverwriteSection(-1, sectionBegin, splitPoint, section.second.laneCount, section.second.offsetx2);
                profile2.OverwriteSection(-1, 0, sectionEnd - splitPoint, section.second.laneCount, section.second.offsetx2);
            }
            else if (splitPoint <= sectionEnd)
            {
                profile2.OverwriteSection(-1, sectionBegin - splitPoint, sectionEnd - splitPoint,
                    section.second.laneCount, section.second.offsetx2);
            }
        }

        for (auto section : oldProfile.GetAllSections(oldLength, 1))
        {
            type_s sectionBegin = section.first.first;
            type_s sectionEnd = section.first.second;
            if (sectionBegin <= splitPoint)
            {
                profile1.OverwriteSection(1, sectionBegin, sectionEnd, section.second.laneCount, section.second.offsetx2);
            }
            else if (sectionEnd < splitPoint && splitPoint < sectionBegin)
            {
                profile1.OverwriteSection(1, splitPoint, sectionEnd, section.second.laneCount, section.second.offsetx2);
                profile2.OverwriteSection(1, sectionBegin - splitPoint, 0, section.second.laneCount, section.second.offsetx2);
            }
            else
            {
                profile2.OverwriteSection(1, sectionBegin - splitPoint, sectionEnd - splitPoint, 
                    section.second.laneCount, section.second.offsetx2);
            }
        }

        auto refLine2 = roadAsPrev->RefLine().split(s);
        auto part2 = std::make_shared<Road>(profile2, refLine2);
        roadAsPrev->profile = profile1;
        roadAsPrev->Generate();

        if (roadAsPrev->successorJunction != nullptr)
        {
            roadAsPrev->successorJunction->NotifyPotentialChange(ChangeInConnecting
                { roadAsPrev, RoadRunner::ChangeInConnecting::Type_DetachAtEnd_Temp });
            roadAsPrev->successorJunction->Attach(RoadRunner::ConnectionInfo{ part2, odr::RoadLink::ContactPoint_End });
        }
#ifndef G_TEST       
        // Move to part2
        auto sm1_it = roadAsPrev->s_to_section_graphics.lower_bound(s);
        std::set<double> sMovedFromPart1;
        for (auto it = sm1_it; it != roadAsPrev->s_to_section_graphics.end(); ++it)
        {
            auto graphics = std::move(it->second);
            graphics->road = part2;
            graphics->sBegin -= s;
            graphics->sEnd -= s;
            part2->s_to_section_graphics.emplace(it->first - s, std::move(graphics));
            sMovedFromPart1.insert(it->first);
        }
        if (sm1_it == roadAsPrev->s_to_section_graphics.end())
        {
            part2->GenerateAllSectionGraphics();
        }
        else
        {
            // Update part2 near split
            double part2CreateLength = std::max(3 * MaxTransition, sm1_it->first - s);
            part2->GenerateOrUpdateSectionGraphicsBetween(0, std::min(part2CreateLength, part2->Length()));
        }

        // Update part1 near split
        for (double sMoved : sMovedFromPart1)
        {
            roadAsPrev->s_to_section_graphics.erase(sMoved);
        }
        roadAsPrev->GenerateOrUpdateSectionGraphicsBetween(std::max(s - 3 * MaxTransition, 0.0), s);
#endif       
        return part2;
    }
}
