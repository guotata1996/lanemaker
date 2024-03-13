#include "road.h"
#include "curve_fitting.h"

#include "junction.h"

namespace RoadRunner
{
    enum RoadJoinError
    {
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
        auto p2 = road2->RefLine().get_xy(0);
        auto f2 = road2->RefLine().get_grad_xy(0);
        auto connectingRef = ConnectLines(p1, odr::normalize(f1), p2, odr::normalize(f2));
        // TODO: special case where p1==f1 and f1==f2
        if (connectingRef == nullptr)
        {
            return RoadJoin_ConnectionInvalidShape;
        }

        type_s linkBase = from_odr_unit(road1->Length());
        type_s road2Base = from_odr_unit(road1->Length() + connectingRef->length);
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
        // Join right profile
        if (road1->profile.HasSide(-1))
        {
            road1->profile.OverwriteSection(-1,
                linkBase, road2Base,
                road1->profile.RightExit().laneCount, road1->profile.RightExit().offsetx2);
            for (const auto& road2section : road2->profile.GetAllSections(from_odr_unit(road2->Length()), -1))
            {
                type_s newStart = road2section.first.first + road2Base;
                type_s newEnd = road2section.first.second + road2Base;
                road1->profile.OverwriteSection(-1, newStart, newEnd,
                    road2section.second.laneCount, road2section.second.offsetx2);
            }
        }
        // Join left profile
        if (road1->profile.HasSide(1))
        {
            road1->profile.OverwriteSection(1,
                road2Base, linkBase,
                road1->profile.LeftEntrance().laneCount, road1->profile.LeftEntrance().offsetx2);

            for (const auto& road2section : road2->profile.GetAllSections(from_odr_unit(road2->Length()), 1))
            {
                type_s newStart = road2section.first.first + road2Base;
                type_s newEnd = road2section.first.second + road2Base;
                road1->profile.OverwriteSection(1, newStart, newEnd,
                    road2section.second.laneCount, road2section.second.offsetx2);
            }
        }

        road1->Generate();
        if (road2->successorJunction != nullptr)
        {
            road2->successorJunction->NotifyPotentialChange(ChangeInConnecting
                { road2, RoadRunner::ChangeInConnecting::Type_DetachAtEnd_Temp });
            road2->successorJunction->Attach(RoadRunner::ConnectionInfo{ road1, odr::RoadLink::ContactPoint_End });
        }
        road2.reset();
        return 0;
    }

    std::shared_ptr<Road> Road::SplitRoad(std::shared_ptr<Road>& roadAsPrev, double s)
    {
        assert(0 < s);
        assert(s < roadAsPrev->Length());
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
        
        return part2;
    }
}
