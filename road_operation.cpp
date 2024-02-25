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

    int JoinRoads(Road* const road1, odr::RoadLink::ContactPoint c1,
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

        auto p1 = road1->generated.ref_line.get_xy(road1->Length());
        auto f1 = road1->generated.ref_line.get_grad_xy(road1->Length());
        auto p2 = road2->generated.ref_line.get_xy(0);
        auto f2 = road2->generated.ref_line.get_grad_xy(0);
        auto connectingRef = ConnectLines(p1, f1, p2, f2);
        if (connectingRef == nullptr)
        {
            return RoadJoin_ConnectionInvalidShape;
        }

        double joinedLength = road1->Length() + road2->Length();
        

        //decltype(road1->profile) joinedProfile(road1-);
    }
}
