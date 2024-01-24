#pragma once

#include "road.h"

#include "spdlog/spdlog.h"

using namespace odr;
using namespace RoadRunner;

const double epsilon = 1e-6;
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
                assert(std::abs(width) < epsilon);
            }
            else
            {
                SPDLOG_INFO("Now at transition{} lane{} whose ptn={}",
                    transitionS, currLaneID, partnerID);

                assert(currLaneID * partnerID > 0);
                double partnerWidth = laneIDToWidth_Next.at(partnerID);
                double partnerSBegin = laneIDToS_Next.at(partnerID);
                assert(std::abs(width - partnerWidth) < epsilon);
                assert(std::abs(sMin - partnerSBegin) < epsilon);
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
                assert(std::abs(width) < epsilon);
            }
            else
            {
                assert(nextLaneID * partnerID > 0);
                double partnerWidth = laneIDToWidth_Curr.at(partnerID);
                double partnerSBegin = laneIDToS_Curr.at(partnerID);
                assert(std::abs(width - partnerWidth) < epsilon);
                assert(std::abs(sMin - partnerSBegin) < epsilon);
            }
        }

        // Verify median lane integrity
        double currMedianWidth = laneIDToS_Curr.at(1);
        double nextMedianWidth = laneIDToS_Next.at(1);
        assert(std::abs(currMedianWidth - nextMedianWidth) < epsilon);

    } // For each section
}

class RoadForRest : public RoadRunner::Road
{
public:
    std::list<RoadRunner::LaneSection> GetLeftProfiles() { return leftProfiles; }
    std::list<RoadRunner::LaneSection> GetRightProfiles() { return rightProfiles; }
};

void VerifySingleRoadIntegrity(RoadRunner::Road config, const odr::Road& gen)
{
    RoadForRest* rt = reinterpret_cast<RoadForRest*>(&config);
    
}