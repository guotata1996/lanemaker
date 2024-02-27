#include "road.h"
#include "curve_fitting.h"

namespace RoadRunner
{
    enum RoadJoinError
    {
        RoadJoin_InputIsIdentical,
        RoadJoin_DirNoOutlet,
        RoadJoin_ConnectionInvalidShape
    };

    int Road::JoinRoads(Road* const road1, odr::RoadLink::ContactPoint c1,
        Road* const road2, odr::RoadLink::ContactPoint c2)
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
        auto connectingRef = ConnectLines(p1, f1, p2, f2);
        if (connectingRef == nullptr)
        {
            return RoadJoin_ConnectionInvalidShape;
        }

        type_s linkBase = from_odr_unit(road1->Length());
        type_s road2Base = from_odr_unit(road1->Length() + connectingRef->length);
        // Connect ref lines
        road1->generated.ref_line.s0_to_geometry.emplace(road1->Length(), connectingRef->clone());
        double baseLength = road1->Length() + connectingRef->length;
        for (const auto& s2geo : road2->RefLine().s0_to_geometry)
        {
            road1->generated.ref_line.s0_to_geometry.emplace(
                baseLength + s2geo.first,
                s2geo.second->clone());
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
        return 0;
    }
}
