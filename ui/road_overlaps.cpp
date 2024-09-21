#include "road_overlaps.h"
#include "world.h"
#include "constants.h"

extern int8_t g_createRoadElevationOption;

namespace RoadRunner
{
    bool TryCreateJunction(std::shared_ptr<RoadRunner::Road> newRoad, 
        double newPartBegin, double newPartEnd,
        std::weak_ptr<RoadRunner::Road> firstCtrlPointPreferredTarget, double firstCtrlPointPreferredS,
        std::weak_ptr<RoadRunner::Road> lastCtrlPointPreferredTarget, double lastCtrlPointPreferredS)
    {
        const double RoadMinLength = 5; // Discard if any leftover road is too short

        bool firstIter = true;
        bool lastTarget = false;
        while (true)
        {
            std::optional<RoadRunner::Road::RoadsOverlap> overlap;
            if (firstIter && !firstCtrlPointPreferredTarget.expired())
            {
                overlap.emplace(newRoad->CalcOverlapWith(firstCtrlPointPreferredTarget.lock(), firstCtrlPointPreferredS, newPartBegin, newPartEnd));
            }
            else
            {
                if (lastTarget)
                {
                    break;
                }
                if (!lastCtrlPointPreferredTarget.expired())
                {
                    overlap.emplace(newRoad->CalcOverlapWith(lastCtrlPointPreferredTarget.lock(), lastCtrlPointPreferredS, newPartBegin, newPartEnd));
                }
                if (!overlap.has_value() || std::abs(overlap->sEnd1 - newPartEnd) > 1e-2)
                {
                    overlap = newRoad->FirstOverlap(newPartBegin, newPartEnd);
                }
                else
                {
                    lastTarget = true;
                }
            }
            firstIter = false;

            if (!overlap.has_value())
            {
                break;
            }

            auto road2 = overlap->road2.lock();

            if (road2 == newRoad)
            {
                spdlog::warn("Self-intersection is currently not supported by junction creation!");
                return false;
            }

            bool canCreateJunction = true; // If not, this collision will end up as an overlap
            double trimBegin1 = std::max(JunctionTrimMin, std::min(JunctionTrimMax, overlap->sBegin1 - RoadMinLength));
            double sBegin1 = overlap->sBegin1 - trimBegin1;

            double trimEnd1 = std::max(JunctionTrimMin, std::min(JunctionTrimMax, newRoad->Length() - overlap->sEnd1 - RoadMinLength));
            double sEnd1 = overlap->sEnd1 + trimEnd1;

            double trimBegin2 = std::max(JunctionTrimMin, std::min(JunctionTrimMax, overlap->sBegin2 - RoadMinLength));
            double sBegin2 = overlap->sBegin2 - trimBegin2;

            double trimEnd2 = std::max(JunctionTrimMin, std::min(JunctionTrimMax, road2->Length() - overlap->sEnd2 - RoadMinLength));
            double sEnd2 = overlap->sEnd2 + trimEnd2;

            if (sBegin1 < RoadMinLength)
            {
                if (newRoad->predecessorJunction != nullptr)
                {
                    // Don't make junction if one is already too close
                    canCreateJunction = false;
                }
                else
                {
                    sBegin1 = 0; // T junction
                }
            }
            if (sEnd1 > newRoad->Length() - RoadMinLength)
            {
                if (newRoad->successorJunction != nullptr)
                {
                    // Don't make junction if one is already too close
                    canCreateJunction = false;
                }
                else
                {
                    sEnd1 = newRoad->Length();  // T junction
                }
            }

            bool joinExistingJunction = road2->generated.junction != "-1";

            if (!joinExistingJunction)
            {
                if (sBegin2 < RoadMinLength)
                {
                    if (road2->predecessorJunction != nullptr)
                    {
                        // Don't make junction if one is already too close
                        canCreateJunction = false;
                    }
                    else
                    {
                        sBegin2 = 0;
                    }
                }
                if (sEnd2 > road2->Length() - RoadMinLength)
                {
                    if (road2->successorJunction != nullptr)
                    {
                        // Don't make junction if one is already too close
                        canCreateJunction = false;
                    }
                    else
                    {
                        sEnd2 = road2->Length();
                    }
                }

                if (sBegin1 == 0 && sEnd1 == newRoad->Length()
                    || sBegin2 == 0 && sEnd2 == road2->Length())
                {
                    spdlog::warn("Road is too short! Cannot create Junction with Road{} @{}", road2->ID(), sEnd1);
                    return false;
                }
            }

            if (!canCreateJunction)
            {
                spdlog::warn("Existing junction is too close! Please switch to bridge/tunnel mode.");
                return false;
            }

            // Can create junction
            std::shared_ptr<RoadRunner::Road> newRoadBeforeJunction, newRoadPastJunction;

            if (sEnd1 != newRoad->Length())
            {
                newRoadPastJunction = RoadRunner::Road::SplitRoad(newRoad, sEnd1);
                World::Instance()->allRoads.insert(newRoadPastJunction);
            }
            if (sBegin1 != 0)
            {
                RoadRunner::Road::SplitRoad(newRoad, sBegin1);
                newRoadBeforeJunction = newRoad;
            }
            else
            {
                World::Instance()->allRoads.erase(newRoad);
            }

            if (joinExistingJunction)
            {
                auto junctionPtr = IDGenerator::ForJunction()->GetByID(road2->generated.junction);
                auto junction = static_cast<RoadRunner::Junction*>(junctionPtr)->shared_from_this();
                road2.reset(); // since road2 is a connecting road inside junction, must reset to free its ID
                if (newRoadBeforeJunction != nullptr)
                {
                    auto errorCode = junction->Attach(RoadRunner::ConnectionInfo{ newRoadBeforeJunction, odr::RoadLink::ContactPoint_End });
                    if (errorCode != RoadRunner::Junction_NoError)
                    {
                        return false;
                    }
                }
                else if (newRoadPastJunction != nullptr)
                {
                    auto errorCode = junction->Attach(RoadRunner::ConnectionInfo{ newRoadPastJunction, odr::RoadLink::ContactPoint_Start });
                    if (errorCode != RoadRunner::Junction_NoError)
                    {
                        return false;
                    }
                }
                else
                {
                    spdlog::warn("Junctions too close or road too short to join existing junction!");
                }
                // New road beyond junction will be trimmed, unless it starts from the junction
                if (sBegin1 != 0 && newRoadPastJunction != nullptr)
                {
                    World::Instance()->allRoads.erase(newRoadPastJunction);
                    break;
                }
            }
            else
            {
                // Elevation is preserved from existing road
                auto junctionElevation = road2->RefLine().elevation_profile.get((sBegin2 + sEnd2) / 2);

                std::shared_ptr<RoadRunner::Road> road2BeforeJunction, road2PastJunction;
                if (sEnd2 != road2->Length())
                {
                    road2PastJunction = RoadRunner::Road::SplitRoad(road2, sEnd2);
                    World::Instance()->allRoads.insert(road2PastJunction);
                }
                if (sBegin2 != 0)
                {
                    RoadRunner::Road::SplitRoad(road2, sBegin2);
                    road2BeforeJunction = road2;
                }
                else
                {
                    World::Instance()->allRoads.erase(road2);
                }

                std::vector<RoadRunner::ConnectionInfo> junctionInfo;

                if (newRoadBeforeJunction != nullptr)
                {
                    junctionInfo.push_back(RoadRunner::ConnectionInfo{ newRoadBeforeJunction, odr::RoadLink::ContactPoint_End });
                }
                if (newRoadPastJunction != nullptr)
                {
                    junctionInfo.push_back(RoadRunner::ConnectionInfo{ newRoadPastJunction, odr::RoadLink::ContactPoint_Start });
                }
                if (road2BeforeJunction != nullptr)
                {
                    junctionInfo.push_back(RoadRunner::ConnectionInfo{ road2BeforeJunction, odr::RoadLink::ContactPoint_End });
                }
                if (road2PastJunction != nullptr)
                {
                    junctionInfo.push_back(RoadRunner::ConnectionInfo{ road2PastJunction, odr::RoadLink::ContactPoint_Start });
                }
                if (junctionInfo.size() < 3)
                {
                    // 2-road junction, should really be a Join
                    canCreateJunction = false;
                }

                for (const auto& conn : junctionInfo)
                {
                    auto connRoad = conn.road.lock();
                    auto connS = conn.contact == odr::RoadLink::ContactPoint_Start ? 0 : connRoad->Length();
                    RoadRunner::CubicSplineGenerator::OverwriteSection(connRoad->RefLine().elevation_profile,
                        connRoad->Length(), connS, connS, junctionElevation);
                    connRoad->GenerateOrUpdateSectionGraphicsBetween(
                        std::max(connS - RoadRunner::CubicSplineGenerator::MaxTransitionLength, 0.0),
                        std::min(connS + RoadRunner::CubicSplineGenerator::MaxTransitionLength, connRoad->Length()));
                }
                auto junction = std::make_shared<RoadRunner::Junction>();
                auto errorCode = junction->CreateFrom(junctionInfo);
                if (errorCode != RoadRunner::Junction_NoError)
                {
                    spdlog::warn("Junction generation reports error code {}", errorCode);
                    return false;
                }
            }

            if (newRoadPastJunction == nullptr)
            {
                break;
            }

            newRoad = newRoadPastJunction;
            newPartBegin = 0;
            newPartEnd -= sEnd1;
        }
        return true;
    }

    bool TryCreateBridgeAndTunnel(std::shared_ptr<RoadRunner::Road> newRoad, double newPartBegin, double newPartEnd)
    {
        auto newPartBeginS = RoadRunner::from_odr_unit(newPartBegin);
        auto newPartEndS = RoadRunner::from_odr_unit(newPartEnd);

        auto allOverlaps = newRoad->AllOverlaps(newPartBegin, newPartEnd);
        if (allOverlaps.empty())
        {
            return true;
        }

        const double clearance = 5;
        auto& existingProfile = newRoad->RefLine().elevation_profile;
        auto  newRoadLength = RoadRunner::from_odr_unit(newRoad->Length());
        for (const auto& overlap : allOverlaps)
        {
            const auto& overlappingProfile = overlap.road2.lock()->RefLine().elevation_profile;

            if (g_createRoadElevationOption > 0)
            {
                auto existingClosest = existingProfile.get_min(overlap.sBegin1, overlap.sEnd1);
                auto existingSafest = existingProfile.get_max(overlap.sBegin1, overlap.sEnd1);
                double overlapRequirement = overlappingProfile.get_max(overlap.sBegin2, overlap.sEnd2) + clearance;
                if (existingClosest < overlapRequirement)
                {
                    RoadRunner::CubicSplineGenerator::OverwriteSection(existingProfile, newRoad->Length(),
                        overlap.sBegin1, overlap.sEnd1, std::max(existingSafest, overlapRequirement));
                }
            }
            else if (g_createRoadElevationOption < 0)
            {
                auto existingClosest = existingProfile.get_max(overlap.sBegin1, overlap.sEnd1);
                auto existingSafest = existingProfile.get_min(overlap.sBegin1, overlap.sEnd1);
                double overlapRequirement = overlappingProfile.get_min(overlap.sBegin2, overlap.sEnd2) - clearance;
                if (existingClosest > overlapRequirement)
                {
                    RoadRunner::CubicSplineGenerator::OverwriteSection(existingProfile, newRoad->Length(),
                        overlap.sBegin1, overlap.sEnd1, std::min(existingSafest, overlapRequirement));
                }
            }
        }


        for (const auto& overlap : allOverlaps)
        {
            newRoad->GenerateOrUpdateSectionGraphicsBetween(
                std::max(overlap.sBegin1 - RoadRunner::CubicSplineGenerator::MaxTransitionLength, 0.0),
                std::min(overlap.sEnd1 + RoadRunner::CubicSplineGenerator::MaxTransitionLength, newRoad->Length()));
        }

        return true;
    }
}