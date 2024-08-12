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

        if (road1->generated.rr_profile.RightExit().laneCount > 0 != road2->generated.rr_profile.RightEntrance().laneCount > 0
            || road2->generated.rr_profile.LeftExit().laneCount > 0 != road1->generated.rr_profile.LeftEntrance().laneCount > 0)
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
            auto connectingRef = ConnectRays(p1, odr::normalize(f1), p2, odr::normalize(f2));
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
        road1->generated.rr_profile.Join(linkBase, road2Base,
            from_odr_unit(road2->Length()), road2->generated.rr_profile, from_odr_unit(road1->Length()));
        road1->generated.rr_eprofile.Join(linkBase, road2Base,
            from_odr_unit(road2->Length()), road2->generated.rr_eprofile, from_odr_unit(road1->Length()));
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
            auto successorJunction = road2->successorJunction;
            ConnectionInfo currInfo(road2, odr::RoadLink::ContactPoint_End);
            successorJunction->FillConnectionInfo(currInfo);
            successorJunction->NotifyPotentialChange(ChangeInConnecting
                { road2, RoadRunner::ChangeInConnecting::Type_DetachAtEnd_Temp });
            successorJunction->Attach(RoadRunner::ConnectionInfo{ 
                road1, odr::RoadLink::ContactPoint_End, currInfo.skipProviderLanes });
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
        ConnectionInfo succJunctionInfo(roadAsPrev, odr::RoadLink::ContactPoint_End);

        type_s oldLength = from_odr_unit(roadAsPrev->Length());
        LaneProfile profile1, profile2;
        roadAsPrev->generated.rr_profile.Split(oldLength, from_odr_unit(s), profile1, profile2);
        ElevationProfile eProfile1, eProfile2;
        roadAsPrev->generated.rr_eprofile.Split(oldLength, from_odr_unit(s), eProfile1, eProfile2);

        auto refLine2 = roadAsPrev->RefLine().split(s);
        auto part2 = std::make_shared<Road>(profile2, eProfile2, refLine2);
        roadAsPrev->generated.rr_profile = profile1;
        roadAsPrev->generated.rr_eprofile = eProfile1;
        roadAsPrev->Generate(false);

        if (roadAsPrev->successorJunction != nullptr)
        {
            auto successorJunction = roadAsPrev->successorJunction;
            successorJunction->FillConnectionInfo(succJunctionInfo);
            successorJunction->NotifyPotentialChange(ChangeInConnecting
                { roadAsPrev, RoadRunner::ChangeInConnecting::Type_DetachAtEnd_Temp });
            successorJunction->Attach(RoadRunner::ConnectionInfo{ 
                part2, odr::RoadLink::ContactPoint_End, succJunctionInfo.skipProviderLanes });
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

    bool Road::ModifyProfile(double s1, double s2,
        const LanePlan& newLeftProfile, const LanePlan& newRightProfile)
    {
        if (s1 > s2) std::swap(s1, s2);

        generated.rr_profile.OverwriteSection(s1, s2, from_odr_unit(Length()), newLeftProfile, newRightProfile);
        Generate(false);

        if (s1 == 0 && predecessorJunction != nullptr)
        {
            predecessorJunction->NotifyPotentialChange();
            if (predecessorJunction->generationError != Junction_NoError)
            {
                return false;
            }
        }
        if (s2 == Length() && successorJunction != nullptr)
        {
            successorJunction->NotifyPotentialChange();
            if (successorJunction->generationError != Junction_NoError)
            {
                return false;
            }
        }

#ifndef G_TEST   
        GenerateOrUpdateSectionGraphicsBetween(
            std::max(s1 - 3 * MaxTransition, 0.0),
            std::min(s2 + 3 * MaxTransition, Length()));
#endif
        return true;
    }
}
