#include "road.h"

#include <algorithm>
#include <vector>

#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"

#include "spdlog/spdlog.h"

namespace RoadRunnder 
{
    void Road::AddLeftSection(const LaneSection& section)
    {
        if (leftProfiles.empty() || leftProfiles.back().profile != section.profile)
        {
            leftProfiles.push_back(section);
        }
        // _UpdateBoundary();
    }

    void Road::AddRightSection(const LaneSection& section)
    {
        if (rightProfiles.empty() || rightProfiles.back().profile != section.profile)
        {
            rightProfiles.push_back(section);
        }
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

    void Road::ConvertSide(bool rightSide,
        std::map<double, odr::LaneSection>& laneSectionResult, 
        std::map<double, odr::Poly3>& laneOffsetResult) const
    {
        const std::list<LaneSection>& profiles = rightSide ? rightProfiles : leftProfiles;

        std::list<LaneSection>::const_iterator pre = profiles.begin(), curr = profiles.begin(), next = profiles.begin();
        curr++;
        next++; next++;

        std::vector<TransitionInfo> transitions(profiles.size());
        transitions[0] = TransitionInfo
        {
            0,
            profiles.front().profile.offsetx2, profiles.front().profile.offsetx2,
            profiles.front().profile.laneCount, 0, 0,
            0
        };  // first element is dummy
        int transitionIndex = 1;
        for (; curr != profiles.end(); ++pre, ++curr, ++transitionIndex)
        {
            const LaneSection& preSection = *pre;
            const LaneSection& currSection = *curr;
            const RoadProfile& preProfile = preSection.profile;
            const RoadProfile& currProfile = currSection.profile;

            // pre->curr
            int newLanesOnLeft = 0, newLanesOnRight = 0;
            if (preProfile.laneCount > currProfile.laneCount)
            {
                // lanes merging
                int vanishedLanes = preProfile.laneCount - currProfile.laneCount;
                type_t leftReduction2 = currProfile.offsetx2 - preProfile.offsetx2;
                type_t rightReduction2 = (preProfile.offsetx2 + preProfile.laneCount * 2) - 
                    (currProfile.offsetx2 + currProfile.laneCount * 2);
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
            else if (preProfile.laneCount < currProfile.laneCount)
            {
                // lane expanding
                int expandedLanes = currProfile.laneCount - preProfile.laneCount;
                type_t leftExpansion2 = preProfile.offsetx2 - currProfile.offsetx2;
                type_t rightExpansion2 = (currProfile.offsetx2 + currProfile.laneCount * 2) - 
                    (preProfile.offsetx2 + preProfile.laneCount * 2);
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

            type_s preLength = currSection.s - preSection.s;
            type_s nextLength = (next == profiles.end() ? Length() : next->s) - currSection.s;
            type_s preTransitionLength = std::min({ preLength / 2, nextLength / 2, MaxTransitionS });

            // write
            transitions[transitionIndex] = TransitionInfo
            {
                currSection.s,
                preProfile.offsetx2, currProfile.offsetx2,
                preProfile.laneCount, newLanesOnLeft , newLanesOnRight,
                preTransitionLength
            };
        }

        for (int i = 0; i != transitions.size(); ++i)
        {
            const TransitionInfo& transition = transitions[i];
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
                    uint32_t laneIndex = 0;
                    odr::LaneSection transitionSection(roadID, to_odr_unit(tranS));
                    odr::Lane center(roadID, to_odr_unit(tranS), laneIndex--, false, "");
                    transitionSection.id_to_lane.emplace(center.id, center);

                    for (int i = 0; i != std::abs(transition.newLanesOnLeft); ++i)
                    {
                        odr::Lane leftVarying(roadID, to_odr_unit(tranS), laneIndex--, false, "driving");

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
                        odr::Lane nonVarying(roadID, to_odr_unit(tranS), laneIndex--, false, "driving");
                        nonVarying.lane_width.s0_to_poly.emplace(to_odr_unit(tranS), nonVaryingWidth);
                        transitionSection.id_to_lane.emplace(nonVarying.id, nonVarying);
                    }

                    for (int i = 0; i != std::abs(transition.newLanesOnRight); ++i)
                    {
                        odr::Lane rightVarying(roadID, to_odr_unit(tranS), laneIndex--, false, "driving");
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
                    laneSectionResult[to_odr_unit(tranS)] = transitionSection;
                }
            }

            if (i == 0 || transition.newLanesOnLeft != 0 || transition.newLanesOnRight != 0)
            {
                uint32_t laneIndex = 0;
                odr::LaneSection straightSection(roadID, to_odr_unit(straightS));
                odr::Lane center(roadID, to_odr_unit(straightS), laneIndex--, false, "");
                straightSection.id_to_lane.emplace(center.id, center);

                odr::Poly3 constWidth(to_odr_unit(straightS), LaneWidth, 0, 0, 0);
                for (int i = 0; i < transition.startLanes + transition.newLanesOnLeft + transition.newLanesOnRight; ++i)
                {
                    odr::Lane nonVarying(roadID, to_odr_unit(straightS), laneIndex--, false, "driving");
                    nonVarying.lane_width.s0_to_poly.emplace(to_odr_unit(straightS), constWidth);
                    straightSection.id_to_lane.emplace(nonVarying.id, nonVarying);
                }
                laneSectionResult[to_odr_unit(straightS)] = straightSection;
            }
        }
    }

    Road::operator odr::Road() const
    {
        double rtnLength = to_odr_unit(Length());
        odr::Road rtn(roadID, rtnLength, "-1", "road " + roadID);
        rtn.ref_line.s0_to_geometry[0] = std::make_unique<odr::Line>(0, 0, 0, 0, rtnLength);

        std::map<double, odr::LaneSection> leftSection, rightSection;
        std::map<double, odr::Poly3> leftOffset, rightOffset;

        ConvertSide(true, rightSection, rightOffset);
        // ConvertSide(false, leftSection, leftOffset);
        // Merge left&right side
        rtn.lane_offset.s0_to_poly = rightOffset;
        rtn.s_to_lanesection = rightSection;

        return rtn;
    }
}