#include "road.h"

#include <algorithm>
#include <vector>

#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"

#include "spdlog/spdlog.h"

namespace RoadRunnder 
{
    void Road::AddSection(const LaneSection& section)
    {
        if (!profiles.empty() && profiles.back().profile == section.profile)
        {
            profiles.back().length += section.length;
        }
        else
        {
            profiles.push_back(section);
        }

        // _UpdateBoundary();
    }

    type_s Road::Length() const
    {
        type_s rtn = 0;
        std::for_each(profiles.begin(), profiles.end(), 
            [&rtn](const LaneSection& s) {rtn += s.length; });
        return rtn;
    }

    double Road::LaneWidth = 3.25;

    std::map<double, odr::Poly3> Road::_MakeTransition(
        type_s start_s, type_s end_s,
        type_t start_t2, type_t end_t2)
    {
        double c_50 = 3.9e-3, d_50 = -5.2e-5;
        double xSpan = to_odr_unit(end_s - start_s);
        double xScale = 50 / xSpan;
        
        double c = c_50 * xScale * xScale;
        double d = d_50 * xScale * xScale * xScale;

        double yStart = to_odr_unit(start_t2);
        double yEnd = to_odr_unit(end_t2);
        // by this point (a,b,c,d) generates 0 --> LaneWidth
        // but we need (yStart --> yEnd)
        double yScale = LaneWidth / (yEnd - yStart);
        double yOffset = yStart;
        double a = yOffset;
        double b = 0;
        c *= yScale;
        d *= yScale;

        return {std::make_pair(
            to_odr_unit(start_s), 
            odr::Poly3(to_odr_unit(start_s), a, b, c, d))
        };
    }

    Road::operator odr::Road() const
    {
        const std::string RoadID = "1";
        double rtnLength = to_odr_unit(Length());
        odr::Road rtn(RoadID, rtnLength, "-1", "road" + RoadID);
        rtn.ref_line.s0_to_geometry[0] = std::make_unique<odr::Line>(0, 0, 0, 0, rtnLength);

        std::list<std::pair<type_s, type_t>> centerDiscrete;
        
        std::transform(profiles.begin(), profiles.end(), std::back_inserter(centerDiscrete), 
            [](const LaneSection& s)
        {
            return std::make_pair(s.length, s.profile.ol2);
        });

        std::list<LaneSection>::const_iterator pre = profiles.begin(), curr = profiles.begin();
        curr++;

        std::vector<TransitionInfo> rightTransitions(profiles.size());
        rightTransitions[0] = TransitionInfo
        {
            0,
            profiles.front().profile.or2, profiles.front().profile.or2,
            profiles.front().profile.nr, 0, 0,
            0
        };  // first element is dummy
        type_s cumulativeS = profiles.front().length;
        int transitionIndex = 1;
        for (; curr != profiles.end(); cumulativeS += curr->length, ++pre, ++curr, ++transitionIndex)
        {
            const LaneSection& preSection = *pre;
            const LaneSection& currSection = *curr;
            const RoadProfile& preProfile = preSection.profile;
            const RoadProfile& currProfile = currSection.profile;
            
            // pre->curr
            int newLanesOnLeft = 0, newLanesOnRight = 0;
            if (preProfile.nr > currProfile.nr)
            {
                // lanes merging
                int vanishedLanes = preProfile.nr - currProfile.nr;
                type_t leftReduction2 = currProfile.or2 - preProfile.or2;
                type_t rightReduction2 = (preProfile.or2 + preProfile.nr * 2) - (currProfile.or2 + currProfile.nr * 2);
                for (int i = 0; i != vanishedLanes; ++i)
                {
                    if (leftReduction2 > rightReduction2)
                    {
                        newLanesOnLeft--;
                        leftReduction2 -= 2;
                    }
                    else
                    {
                        newLanesOnRight--;
                        rightReduction2 -= 2;
                    }
                }
            }
            else if (preProfile.nr < currProfile.nr)
            {
                // lane expanding
                int expandedLanes = currProfile.nr - preProfile.nr;
                type_t leftExpansion2 = preProfile.or2 - currProfile.or2;
                type_t rightExpansion2 = (currProfile.or2 + currProfile.nr * 2) - (preProfile.or2 + preProfile.nr * 2);
                for (int i = 0; i != expandedLanes; ++i)
                {
                    if (leftExpansion2 > rightExpansion2)
                    {
                        newLanesOnLeft++;
                        leftExpansion2 -= 2;
                    }
                    else
                    {
                        newLanesOnRight++;
                        rightExpansion2 -= 2;
                    }
                }
            }
            type_s preTransitionLength = std::min({ preSection.length / 2, currSection.length / 2, MaxTransitionS });
            
            // write
            rightTransitions[transitionIndex] = TransitionInfo
            {
                cumulativeS,
                preProfile.or2, currProfile.or2,
                preProfile.nr, newLanesOnLeft , newLanesOnRight, 
                preTransitionLength 
            };
        }

        std::map<double, odr::Poly3>& laneOffsetResult = rtn.lane_offset.s0_to_poly;

        for (int i = 0; i != rightTransitions.size(); ++i)
        {
            const TransitionInfo& transition = rightTransitions[i];
            type_s tranS = transition.cumulativeS - transition.transitionHalfLength;
            type_s straightS = transition.cumulativeS + transition.transitionHalfLength;

            if (transition.oldCenter2 != transition.newCenter2)
            {
                for (auto laneOffsetTransition : _MakeTransition(
                    tranS, straightS,
                    -transition.oldCenter2, -transition.newCenter2))
                {
                    laneOffsetResult.insert(laneOffsetTransition);
                }
            }
            if (transition.oldCenter2 != transition.newCenter2 || i == 0)
            {
                odr::Poly3 straight(
                    to_odr_unit(straightS),
                    to_odr_unit((type_t)-transition.newCenter2),
                    0, 0, 0
                );
                laneOffsetResult.emplace(to_odr_unit(straightS), straight);
            }

            SPDLOG_INFO("Right transition {}: s={}, {} | {} | {}",
                i, transition.cumulativeS, transition.newLanesOnLeft, transition.startLanes, transition.newLanesOnRight);

            if (transition.newLanesOnLeft != 0 || transition.newLanesOnRight != 0)
            {
                {
                    // varying section
                    uint32_t rightLaneIndex = 0;
                    odr::LaneSection transitionSection(RoadID, to_odr_unit(tranS));
                    odr::Lane center(RoadID, to_odr_unit(tranS), rightLaneIndex--, false, "");
                    transitionSection.id_to_lane.emplace(center.id, center);

                    for (int i = 0; i != std::abs(transition.newLanesOnLeft); ++i)
                    {
                        odr::Lane leftVarying(RoadID, to_odr_unit(tranS), rightLaneIndex--, false, "driving");

                        for (auto transitionWidth : _MakeTransition(
                            tranS, straightS,
                            transition.newLanesOnLeft > 0 ? 0 : 2,
                            transition.newLanesOnLeft > 0 ? 2 : 0))
                        {
                            leftVarying.lane_width.s0_to_poly.insert(transitionWidth);
                        }
                        transitionSection.id_to_lane.emplace(leftVarying.id, leftVarying);
                    }

                    odr::Poly3 nonVaryingWidth(to_odr_unit(tranS), LaneWidth, 0, 0, 0);
                    for (int i = 0; i != std::min(transition.startLanes, 
                        transition.startLanes + transition.newLanesOnLeft + transition.newLanesOnRight); 
                        ++i)
                    {
                        odr::Lane nonVarying(RoadID, to_odr_unit(tranS), rightLaneIndex--, false, "driving");
                        nonVarying.lane_width.s0_to_poly.emplace(to_odr_unit(tranS), nonVaryingWidth);
                        transitionSection.id_to_lane.emplace(nonVarying.id, nonVarying);
                    }

                    for (int i = 0; i != std::abs(transition.newLanesOnRight); ++i)
                    {
                        odr::Lane rightVarying(RoadID, to_odr_unit(tranS), rightLaneIndex--, false, "driving");
                        for (auto transitionWidth : _MakeTransition(
                            tranS, straightS,
                            transition.newLanesOnRight > 0 ? 0 : 2,
                            transition.newLanesOnRight > 0 ? 2 : 0
                        ))
                        {
                            rightVarying.lane_width.s0_to_poly.insert(transitionWidth);
                        }
                        transitionSection.id_to_lane.emplace(rightVarying.id, rightVarying);
                    }
                    rtn.s_to_lanesection[to_odr_unit(tranS)] = transitionSection;
                }
            }

            if (i == 0 || transition.newLanesOnLeft != 0 || transition.newLanesOnRight != 0)
            {
                uint32_t rightLaneIndex = 0;
                odr::LaneSection straightSection(RoadID, to_odr_unit(straightS));
                odr::Lane center(RoadID, to_odr_unit(straightS), rightLaneIndex--, false, "");
                straightSection.id_to_lane.emplace(center.id, center);

                odr::Poly3 constWidth(to_odr_unit(straightS), LaneWidth, 0, 0, 0);
                for (int i = 0; i < transition.startLanes + transition.newLanesOnLeft + transition.newLanesOnRight; ++i)
                {
                    odr::Lane nonVarying(RoadID, to_odr_unit(straightS), rightLaneIndex--, false, "driving");
                    nonVarying.lane_width.s0_to_poly.emplace(to_odr_unit(straightS), constWidth);
                    straightSection.id_to_lane.emplace(nonVarying.id, nonVarying);
                }
                rtn.s_to_lanesection[to_odr_unit(straightS)] = straightSection;
            }
        }
        return rtn;
    }
}