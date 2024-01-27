#pragma once

#include "road.h"

#include "spdlog/spdlog.h"

#ifdef G_TEST
#include <gtest/gtest.h>
#endif

using namespace odr;
using namespace RoadRunner;

const double epsilon = 1e-6;

void VerifyLaneWidthinBound(const odr::Road& road)
{
    constexpr int checkPoints = 15;
    auto sectionStart = road.s_to_lanesection.begin();
    for (;sectionStart != road.s_to_lanesection.end(); sectionStart++)
    {
        double sStart = sectionStart->first;
        decltype(sectionStart) sectionEnd = sectionStart;
        sectionEnd++;
        double sEnd = sectionEnd == road.s_to_lanesection.end() ? road.length : sectionEnd->first;
        auto section = sectionStart->second;
        for (auto idToLane : section.id_to_lane)
        {
            if (idToLane.first == 0)
            {
                for (int i = 0; i <= checkPoints; ++i)
                {
                    double s = sStart + (sEnd - sStart) * (double)i / checkPoints;
                    double width = idToLane.second.lane_width.get(s);
#ifdef G_TEST
                    EXPECT_EQ(width, 0);
#endif
                }
            }
            else if (idToLane.first == 1)
            {
                for (int i = 0; i <= checkPoints; ++i)
                {
                    double s = sStart + (sEnd - sStart) * (double)i / checkPoints;
                    double width = idToLane.second.lane_width.get(s);
#ifdef G_TEST
                    EXPECT_LT(-epsilon, width);
#endif
                }
            }
            else
            {
                for (int i = 0; i <= checkPoints; ++i)
                {
                    double s = sStart + (sEnd - sStart) * (double)i / checkPoints;
                    double width = idToLane.second.lane_width.get(s);
#ifdef G_TEST
                    EXPECT_LT(-epsilon, width);
                    EXPECT_LT(width, RoadRunner::Road::LaneWidth + epsilon);
#endif
                }
            }
        }
    }
}

void VerifySingleRoadLinkage(const odr::Road& road)
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
#ifdef G_TEST
                EXPECT_NEAR(width, 0, epsilon);
#endif
            }
            else
            {
                SPDLOG_TRACE("Now at transition{} lane{} whose ptn={}",
                    transitionS, currLaneID, partnerID);
                double partnerWidth = laneIDToWidth_Next.at(partnerID);
                double partnerSBegin = laneIDToS_Next.at(partnerID);
#ifdef G_TEST
                EXPECT_GT(currLaneID * partnerID, 0);
                EXPECT_NEAR(width, partnerWidth, epsilon);
                EXPECT_NEAR(sMin, partnerSBegin, epsilon);
#endif
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
#ifdef G_TEST
                EXPECT_NEAR(width, 0, epsilon);
#endif
            }
            else
            {
                double partnerWidth = laneIDToWidth_Curr.at(partnerID);
                double partnerSBegin = laneIDToS_Curr.at(partnerID);
#ifdef G_TEST
                EXPECT_GT(nextLaneID* partnerID, 0);
                EXPECT_NEAR(width, partnerWidth, epsilon);
                EXPECT_NEAR(sMin, partnerSBegin, epsilon);
#endif
            }
        }

        // Verify median lane integrity
        double currMedianWidth = laneIDToS_Curr.at(1);
        double nextMedianWidth = laneIDToS_Next.at(1);
#ifdef G_TEST
        EXPECT_NEAR(currMedianWidth, nextMedianWidth, epsilon);
#endif

    } // For each section
}

class RoadForTest : public RoadRunner::Road
{
public:
    const std::list<RoadRunner::LaneSection>& GetLeftProfiles() { return leftProfiles; }
    const std::list<RoadRunner::LaneSection>& GetRightProfiles() { return rightProfiles; }
};

void VerifySingleRoadIntegrity(RoadRunner::Road configs, const odr::Road& gen)
{
    RoadForTest* rt = reinterpret_cast<RoadForTest*>(&configs);

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
    auto sToSectionIt = rt->GetRightProfiles().begin();

    for (auto probe = allProbeS.begin(); 
        probe != allProbeS.end() && sToSectionIt != rt->GetRightProfiles().end(); 
        probe++ )
    {
        type_s s = sToSectionIt->s;
        double odr_s = RoadRunner::Road::to_odr_unit(s);
        auto config = sToSectionIt->profile;
        while (*probe < odr_s)
        {
            // probe location must exceed s
            probe++;
        }
        double rightRim = gen.lane_offset.get(*probe);
        double expectedRightRim = RoadRunner::Road::to_odr_unit(config.offsetx2);
        bool rightRimSatisfy = std::abs(rightRim - expectedRightRim) < epsilon;
        odr::LaneSection probeSection = inverse_s_to_laneSection.lower_bound(-*probe)->second;
        auto rightMost = probeSection.id_to_lane.begin();
        int fullLanes = 0;

        for (int laneID = -1; laneID >= rightMost->first; laneID--)
        {
            odr::Lane lane = probeSection.id_to_lane.at(laneID);
            if (std::abs(lane.lane_width.get(*probe) - RoadRunner::Road::LaneWidth) < epsilon)
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
            SPDLOG_TRACE("Right spec {} is satisfied at {}", odr_s, *probe);
            sToSectionIt++;
        }
        else
        {
            SPDLOG_TRACE("Right spec {} is not satisfied at{} (section {}). Expected offset {} actual {}; Expected lanes {} actual {}",
                odr_s, *probe, probeSection.s0,
                expectedRightRim, rightRim,
                config.laneCount, fullLanes);
        }
    }

#ifdef G_TEST
    EXPECT_EQ(sToSectionIt, rt->GetRightProfiles().end());
#endif

    //////////////////////
    // Left side
    //////////////////////
    sToSectionIt = rt->GetLeftProfiles().begin();

    for (auto probe = allProbeS.rbegin();
        probe != allProbeS.rend() && sToSectionIt != rt->GetLeftProfiles().end();
        probe++)
    {
        type_s s = sToSectionIt->s;
        double odr_s = RoadRunner::Road::to_odr_unit(s);
        auto config = sToSectionIt->profile;
        while (*probe > odr_s)
        {
            probe++;
        }
        double rightRim = gen.lane_offset.get(*probe);

        odr::LaneSection probeSection = inverse_s_to_laneSection.lower_bound(-*probe)->second;
        double medianWidth = probeSection.id_to_lane.at(1).lane_width.get(*probe);
        double leftRim = rightRim + medianWidth;
        double expectedLeftRim = RoadRunner::Road::to_odr_unit(config.offsetx2);
        bool leftRimSatisfy = std::abs(leftRim - expectedLeftRim) < epsilon;

        auto leftMost = probeSection.id_to_lane.rbegin();
        int fullLanes = 0;

        for (int laneID = 2; laneID <= leftMost->first; laneID++)
        {
            odr::Lane lane = probeSection.id_to_lane.at(laneID);
            if (std::abs(lane.lane_width.get(*probe) - RoadRunner::Road::LaneWidth) < epsilon)
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
            SPDLOG_TRACE("Left spec {} is satisfied at {}", odr_s, *probe);
            sToSectionIt++;
        }
        else
        {
            SPDLOG_TRACE("Left spec {} is not satisfied at{} (section {}). Expected offset {} actual {}; Expected lanes {} actual {}",
                odr_s, *probe, probeSection.s0,
                expectedLeftRim, leftRim,
                config.laneCount, fullLanes);
        }
    }

#ifdef G_TEST
    EXPECT_EQ(sToSectionIt, rt->GetLeftProfiles().end());
#endif
}
