#include "road_overlaps.h"
#include "world.h"
#include "constants.h"

#include <spdlog/spdlog.h>

namespace LM
{
    bool CreateJunctionAtZOverlap(std::shared_ptr<LM::Road> newRoad, double newPartBegin, double newPartEnd)
    {
        const double RoadMinLength = 5; // Discard if any leftover road is too short

        while (true)
        {
            std::optional<LM::Road::RoadsOverlap> overlap = newRoad->FirstOverlap(newPartBegin, newPartEnd);

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

            const bool joinExistingJunction = road2->IsConnectingRoad();
            if (sBegin1 == 0 && joinExistingJunction) {
                // Current overlap can be with any connecting road of this junction, but we need to:
                //   Process at the last connecting road of originating junction
                //   Process at the first connecting road of terminal junction
                const double immediatelyPastOverlap = overlap->sEnd1 + 0.01;
                auto nextOverlap = newRoad->FirstOverlap(immediatelyPastOverlap, newPartEnd);
                if (nextOverlap.has_value() &&
                    nextOverlap->road2.lock()->generated.junction == road2->generated.junction)
                {
                    auto newRoadPastJunction = LM::Road::SplitRoad(newRoad, immediatelyPastOverlap);
                    World::Instance()->allRoads.insert(newRoadPastJunction);
                    World::Instance()->allRoads.erase(newRoad);
                    newRoad = newRoadPastJunction;
                    newPartBegin = 0;
                    newPartEnd -= immediatelyPastOverlap;

                    continue;
                }
            }

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
                spdlog::warn("Cannot create junction due to existing one nearby!");
                return false;
            }

            // Can create junction
            std::shared_ptr<LM::Road> newRoadBeforeJunction, newRoadPastJunction;

            if (sEnd1 != newRoad->Length())
            {
                newRoadPastJunction = LM::Road::SplitRoad(newRoad, sEnd1);
                World::Instance()->allRoads.insert(newRoadPastJunction);
            }
            if (sBegin1 != 0)
            {
                LM::Road::SplitRoad(newRoad, sBegin1);
                newRoadBeforeJunction = newRoad;
            }
            else
            {
                World::Instance()->allRoads.erase(newRoad);
            }

            if (joinExistingJunction)
            {
                auto junctionPtr = IDGenerator::ForType(IDType::Junction)->GetByID<LM::Junction>(road2->generated.junction);
                auto junction = junctionPtr->shared_from_this();
                road2.reset(); // since road2 is a connecting road inside junction, must reset to free its ID
                if (newRoadBeforeJunction != nullptr)
                {
                    auto errorCode = junction->Attach(LM::ConnectionInfo{ newRoadBeforeJunction, odr::RoadLink::ContactPoint_End });
                    if (errorCode != LM::Junction_NoError)
                    {
                        return false;
                    }
                }
                else if (newRoadPastJunction != nullptr)
                {
                    auto errorCode = junction->Attach(LM::ConnectionInfo{ newRoadPastJunction, odr::RoadLink::ContactPoint_Start });
                    if (errorCode != LM::Junction_NoError)
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

                std::shared_ptr<LM::Road> road2BeforeJunction, road2PastJunction;
                if (sEnd2 != road2->Length())
                {
                    road2PastJunction = LM::Road::SplitRoad(road2, sEnd2);
                    World::Instance()->allRoads.insert(road2PastJunction);
                }
                if (sBegin2 != 0)
                {
                    LM::Road::SplitRoad(road2, sBegin2);
                    road2BeforeJunction = road2;
                }
                else
                {
                    World::Instance()->allRoads.erase(road2);
                }

                std::vector<LM::ConnectionInfo> junctionInfo;

                if (newRoadBeforeJunction != nullptr)
                {
                    junctionInfo.push_back(LM::ConnectionInfo{ newRoadBeforeJunction, odr::RoadLink::ContactPoint_End });
                }
                if (newRoadPastJunction != nullptr)
                {
                    junctionInfo.push_back(LM::ConnectionInfo{ newRoadPastJunction, odr::RoadLink::ContactPoint_Start });
                }
                if (road2BeforeJunction != nullptr)
                {
                    junctionInfo.push_back(LM::ConnectionInfo{ road2BeforeJunction, odr::RoadLink::ContactPoint_End });
                }
                if (road2PastJunction != nullptr)
                {
                    junctionInfo.push_back(LM::ConnectionInfo{ road2PastJunction, odr::RoadLink::ContactPoint_Start });
                }
                if (junctionInfo.size() < 3)
                {
                    spdlog::warn("Created 2-road junction, should really be a Join");
                }

                for (const auto& conn : junctionInfo)
                {
                    auto connRoad = conn.road.lock();
                    auto connS = conn.contact == odr::RoadLink::ContactPoint_Start ? 0 : connRoad->Length();
                    LM::CubicSplineGenerator::OverwriteSection(connRoad->RefLine().elevation_profile,
                        connRoad->Length(), connS, connS, junctionElevation);
                    connRoad->GenerateOrUpdateSectionGraphicsBetween(0.0, connRoad->Length());
                }
                auto junction = std::make_shared<LM::Junction>();
                auto errorCode = junction->CreateFrom(junctionInfo);
                if (errorCode != LM::Junction_NoError)
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
}