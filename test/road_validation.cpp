#include "validation.h"

#include "road.h"
#include "junction.h"

#include "test_const.h"

#ifdef G_TEST
    #include <gtest/gtest.h>
#endif

using namespace odr;
using namespace RoadRunner;

namespace RoadRunnerTest
{
    void Validation::VerifySingleRoad(const odr::Road& road)
    {
#ifdef G_TEST
        VerifyLaneWidthinBound(road);
#endif
        VerifySingleRoadLinkage(road);
        VerifyProfileIntegrity(road);
        VerifySingleRoadElevation(road.ref_line.elevation_profile);
        VerifyRoadMarking(road);
    }

    void Validation::VerifyLaneWidthinBound(const odr::Road& road)
    {
        constexpr int checkPoints = 15;
        auto sectionStart = road.s_to_lanesection.begin();
        for (; sectionStart != road.s_to_lanesection.end(); sectionStart++)
        {
            double sStart = sectionStart->first;
            decltype(sectionStart) sectionEnd = sectionStart;
            sectionEnd++;
            double sEnd = sectionEnd == road.s_to_lanesection.end() ? road.length : sectionEnd->first;
            auto section = sectionStart->second;
            for (auto idToLane : section.id_to_lane)
            {
                for (int i = 0; i <= checkPoints; ++i)
                {
                    double s = sStart + (sEnd - sStart) * (double)i / checkPoints;
                    double width = idToLane.second.lane_width.get(s);
#ifdef G_TEST
                    if (idToLane.first == 0)
                    {
                        EXPECT_EQ(width, 0);
                    }
                    else if (idToLane.second.type == "median")
                    {
                        EXPECT_LT(-epsilon, width);
                    }
                    else if (idToLane.second.type == "driving")
                    {
                        EXPECT_LT(-epsilon, width);
                        EXPECT_LT(width, RoadRunner::LaneWidth + epsilon);
                    }
#endif
                }
            }
        }
    }

    void Validation::VerifySingleRoadLinkage(const odr::Road& road)
    {
        auto curr = road.s_to_lanesection.cbegin();
        auto next = curr;
        next++;
        for (; next != road.s_to_lanesection.end(); ++curr, ++next)
        {
            auto currIdToLane = curr->second.id_to_lane;
            auto nextIDToLane = next->second.id_to_lane;
            double transitionS = next->first;

            std::map<int, double> laneIDToWidth_Curr, laneIDToS_Curr; // s W.R.T. laneOffset
            std::map<int, double> laneIDToWidth_Next, laneIDToS_Next;

            double cumulativeS = 0;
            // Compute curr R
            for (auto idAndLane = currIdToLane.rbegin(); idAndLane != currIdToLane.rend(); ++idAndLane)
            {
                int laneID = idAndLane->first;
                if (laneID >= 0) continue;

                double currWidth = idAndLane->second.lane_width.get(transitionS);
                // SPDLOG_INFO("Right Lane {} at {} has width {}", idAndLane->first, transitionS, currWidthEnd);
                laneIDToWidth_Curr.emplace(laneID, currWidth);
                laneIDToS_Curr.emplace(laneID, cumulativeS);
                cumulativeS += currWidth;
            }

            // Compute curr L
            cumulativeS = 0;
            for (auto idAndLane = currIdToLane.begin(); idAndLane != currIdToLane.end(); ++idAndLane)
            {
                int laneID = idAndLane->first;
                if (laneID <= 0) continue;

                double currWidth = idAndLane->second.lane_width.get(transitionS);
                laneIDToWidth_Curr.emplace(laneID, currWidth);
                laneIDToS_Curr.emplace(laneID, cumulativeS);
                cumulativeS += currWidth;
            }

            // Compute next R
            cumulativeS = 0;
            for (auto idAndLane = nextIDToLane.rbegin(); idAndLane != nextIDToLane.rend(); ++idAndLane)
            {
                int laneID = idAndLane->first;
                if (laneID >= 0) continue;

                double width = idAndLane->second.lane_width.get(transitionS);
                laneIDToWidth_Next.emplace(laneID, width);
                laneIDToS_Next.emplace(laneID, cumulativeS);
                cumulativeS += width;
            }

            // Compute next L
            cumulativeS = 0;
            for (auto idAndLane = nextIDToLane.begin(); idAndLane != nextIDToLane.end(); ++idAndLane)
            {
                int laneID = idAndLane->first;
                if (laneID <= 0) continue;
                double width = idAndLane->second.lane_width.get(transitionS);
                laneIDToWidth_Next.emplace(laneID, width);
                laneIDToS_Next.emplace(laneID, cumulativeS);
                cumulativeS += width;
            }

            // Verify right side successor and left side predessor
            for (auto idAndWidth : laneIDToWidth_Curr)
            {
                int currLaneID = idAndWidth.first;
                if (currLaneID == 1) continue;  // skip median
                double width = idAndWidth.second;
                double sMin = laneIDToS_Curr.at(currLaneID); // Positive
                Lane currLane = currIdToLane.at(currLaneID);

                int partnerID = currLaneID < 0 ? currLane.successor : currLane.predecessor;
                if (partnerID == 0)
                {
                    ExpectNearOrAssert(width, 0, epsilon);
                }
                else
                {
                    spdlog::debug("Now at transition{} lane{} whose ptn={}",
                        transitionS, currLaneID, partnerID);
                    double partnerWidth = laneIDToWidth_Next.at(partnerID);
                    double partnerSBegin = laneIDToS_Next.at(partnerID);
                    ExpectGTOrAssert(currLaneID * partnerID, 0);
                    ExpectNearOrAssert(width, partnerWidth, epsilon);
                    ExpectNearOrAssert(sMin, partnerSBegin, epsilon);
                }
            }

            // Verify right side predecessor & left side successor
            for (auto idAndWidth : laneIDToWidth_Next)
            {
                int nextLaneID = idAndWidth.first;
                if (nextLaneID == 1) continue;  // skip median
                double width = idAndWidth.second;
                double sMin = laneIDToS_Next.at(nextLaneID);
                Lane nextLane = nextIDToLane.at(nextLaneID);

                int partnerID = nextLaneID < 0 ? nextLane.predecessor : nextLane.successor;
                if (partnerID == 0)
                {
                    ExpectNearOrAssert(width, 0, epsilon);
                }
                else
                {
                    double partnerWidth = laneIDToWidth_Curr.at(partnerID);
                    double partnerSBegin = laneIDToS_Curr.at(partnerID);
                    ExpectGTOrAssert(nextLaneID * partnerID, 0);
                    ExpectNearOrAssert(width, partnerWidth, epsilon);
                    ExpectNearOrAssert(sMin, partnerSBegin, epsilon);
                }
            }

            // Verify median lane integrity
            if (laneIDToS_Curr.find(1) != laneIDToS_Curr.end())
            {
                double currMedianWidth = laneIDToS_Curr.at(1);
                double nextMedianWidth = laneIDToS_Next.at(1);
                ExpectNearOrAssert(currMedianWidth, nextMedianWidth, epsilon);
            }
        } // For each section
    }

    void Validation::VerifyProfileIntegrity(const odr::Road& gen)
    {
        std::set<double> laneSectionKeys = odr::get_map_keys(gen.s_to_lanesection);
        std::set<double> laneOffsetKeys = odr::get_map_keys(gen.lane_offset.s0_to_poly);
        laneSectionKeys.insert(laneOffsetKeys.begin(), laneOffsetKeys.end());
        laneSectionKeys.insert(gen.length);
        std::vector<double> allProbeS(laneSectionKeys.begin(), laneSectionKeys.end());
        std::sort(allProbeS.begin(), allProbeS.end());

        decltype(gen.s_to_lanesection) inverse_s_to_laneSection;
        for (auto it : gen.s_to_lanesection)
        {
            inverse_s_to_laneSection.emplace(-it.first, it.second);
        }

        //////////////////////
        // Right side
        //////////////////////
        std::map<type_s, LanePlan> rProfileCopy;
        for (auto s2Profile : gen.rr_profile.rightPlan)
        {
            // We don't care about out-of-range(0-length) SectionConfigs
            if (to_odr_unit(s2Profile.first) < gen.length - epsilon)
            {
                rProfileCopy.insert(s2Profile);
            }
        }

        auto sToSectionIt = rProfileCopy.begin();

        for (auto probe = allProbeS.begin();
            probe != allProbeS.end() && sToSectionIt != rProfileCopy.end();
            probe++)
        {
            type_s s = sToSectionIt->first;
            double odr_s = RoadRunner::to_odr_unit(s);
            auto config = sToSectionIt->second;
            while (*probe < odr_s)
            {
                // probe location must exceed s
                probe++;
            }
            double rightRim = gen.lane_offset.get(*probe);
            double expectedRightRim = RoadRunner::to_odr_unit(config.offsetx2);
            bool rightRimSatisfy = std::abs(rightRim - expectedRightRim) < epsilon;
            odr::LaneSection probeSection = inverse_s_to_laneSection.lower_bound(-*probe)->second;
            auto rightMost = probeSection.id_to_lane.begin();
            int fullLanes = 0;

            for (int laneID = -1; laneID >= rightMost->first; laneID--)
            {
                odr::Lane lane = probeSection.id_to_lane.at(laneID);
                if (std::abs(lane.lane_width.get(*probe) - RoadRunner::LaneWidth) < epsilon)
                {
                    fullLanes++;
                }
                else if (std::abs(lane.lane_width.get(*probe)) > epsilon)
                {
                    // partial lane is not allowed!
                    fullLanes = -1;
                    break;
                }
            }

            bool rightLaneSatisfy = fullLanes == config.laneCount;
            if (rightRimSatisfy && rightLaneSatisfy)
            {
                spdlog::debug("Right spec {} is satisfied at {}", odr_s, *probe);
                sToSectionIt++;
            }
            else
            {
                spdlog::debug("Right spec {} is not satisfied at{} (section {}). Expected offset {} actual {}; Expected lanes {} actual {}",
                    odr_s, *probe, probeSection.s0,
                    expectedRightRim, rightRim,
                    config.laneCount, fullLanes);
            }
        }

        ExpectOrAssert(sToSectionIt == rProfileCopy.end());

        {
            bool hasMedianLane = !gen.rr_profile.rightPlan.empty();

            //////////////////////
            // Left side
            //////////////////////
            std::map<type_s, LanePlan> lProfileCopy;
            for (auto s2Profile : gen.rr_profile.leftPlans)
            {
                // We don't care about out-of-range(1-length) SectionConfigs
                if (0 < s2Profile.first && to_odr_unit(s2Profile.first) < gen.length - epsilon)
                {
                    lProfileCopy.insert(s2Profile);
                }
            }

            auto sToSectionIt = lProfileCopy.rbegin();

            for (auto probe = allProbeS.rbegin();
                probe != allProbeS.rend() && sToSectionIt != lProfileCopy.rend();
                probe++)
            {
                type_s s = sToSectionIt->first;
                double odr_s = RoadRunner::to_odr_unit(s);
                auto config = sToSectionIt->second;
                while (*probe > odr_s)
                {
                    probe++;
                }
                double rightRim = gen.lane_offset.get(*probe);

                odr::LaneSection probeSection = inverse_s_to_laneSection.lower_bound(-*probe)->second;
                double medianWidth = hasMedianLane ? probeSection.id_to_lane.at(1).lane_width.get(*probe) : 0;
                double leftRim = rightRim + medianWidth;
                double expectedLeftRim = RoadRunner::to_odr_unit(config.offsetx2);
                bool leftRimSatisfy = std::abs(leftRim - expectedLeftRim) < epsilon;

                auto leftMost = probeSection.id_to_lane.rbegin();
                int fullLanes = 0;

                for (int laneID = hasMedianLane ? 2 : 1; laneID <= leftMost->first; laneID++)
                {
                    odr::Lane lane = probeSection.id_to_lane.at(laneID);
                    if (std::abs(lane.lane_width.get(*probe) - RoadRunner::LaneWidth) < epsilon)
                    {
                        fullLanes++;
                    }
                    else if (std::abs(lane.lane_width.get(*probe)) > epsilon)
                    {
                        // partial lane is not allowed!
                        fullLanes = -1;
                        break;
                    }
                }

                bool leftLaneSatisfy = fullLanes == config.laneCount;

                if (leftRimSatisfy && leftLaneSatisfy)
                {
                    spdlog::debug("Left spec {} is satisfied at {}", odr_s, *probe);
                    sToSectionIt++;
                }
                else
                {
                    spdlog::debug("Left spec {} is not satisfied at{} (section {}). Expected offset {} actual {}; Expected lanes {} actual {}",
                        odr_s, *probe, probeSection.s0,
                        expectedLeftRim, leftRim,
                        config.laneCount, fullLanes);
                }
            }

        ExpectOrAssert(sToSectionIt == lProfileCopy.rend());
        }
    }

    void Validation::VerifySingleRoadElevation(const odr::CubicSpline& eProfile)
    {
        for (auto it = eProfile.s0_to_poly.begin(); it != eProfile.s0_to_poly.end(); ++it)
        {
            auto next = it;
            next++;
            if (next == eProfile.s0_to_poly.end()) break;
            auto s = next->first;
            ExpectNearOrAssert(it->second.get(s), next->second.get(s), epsilon);
        }
    }

    void Validation::VerifyRoadMarking(const odr::Road& road)
    {
        {
            bool successorIsCommon = false;
            auto successorStopLineObjectIt = road.id_to_object.find(std::to_string(odr::RoadLink::ContactPoint_End));
            if (road.successor.type == odr::RoadLink::Type_Junction)
            {
                auto nextJunc = static_cast<RoadRunner::AbstractJunction*>(IDGenerator::ForJunction()->GetByID(road.successor.id));
                if (nextJunc->generated.type == odr::JunctionType::Common)
                {
                    successorIsCommon = true;
                }
            }
            if (successorIsCommon)
            {
                ExpectOrAssert(successorStopLineObjectIt != road.id_to_object.end());
                ExpectNearOrAssert(successorStopLineObjectIt->second.s0, road.length, 1);
            }
            else
            {
                ExpectOrAssert(successorStopLineObjectIt == road.id_to_object.end());
            }
        }
        {
            bool predecessorIsCommon = false;
            auto predecessorStopLineObjectIt = road.id_to_object.find(std::to_string(odr::RoadLink::ContactPoint_Start));
            if (road.predecessor.type == odr::RoadLink::Type_Junction)
            {
                auto predJunc = static_cast<RoadRunner::AbstractJunction*>(IDGenerator::ForJunction()->GetByID(road.predecessor.id));
                if (predJunc->generated.type == odr::JunctionType::Common)
                {
                    predecessorIsCommon = true;
                }
            }
            if (predecessorIsCommon)
            {
                ExpectOrAssert(predecessorStopLineObjectIt != road.id_to_object.end());
                ExpectNearOrAssert(predecessorStopLineObjectIt->second.s0, 0, 1);
            }
            else
            {
                ExpectOrAssert(predecessorStopLineObjectIt == road.id_to_object.end());
            }
        }
    }
}
