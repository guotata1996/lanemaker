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
        double yScale = (yEnd - yStart) / LaneWidth;
        double yOffset = yStart;
        double a = yOffset;
        double b = 0;
        c *= yScale;
        d *= yScale;

        odr::Poly3 test(to_odr_unit(start_s), a, b, c, d);

        return { std::make_pair(
            to_odr_unit(start_s),
            odr::Poly3(to_odr_unit(start_s), a, b, c, d))
        };
    }

    void Road::ConvertSide(bool rightSide,
        std::map<double, odr::LaneSection>& laneSectionResult,
        std::map<double, odr::Poly3>& laneOffsetResult) const
    {
        const std::list<LaneSection>& profiles = rightSide ? rightProfiles : leftProfiles;
        const int laneIDMultiplier = rightSide ? -1 : 1;

        std::list<LaneSection>::const_iterator pre = profiles.begin(), curr = profiles.begin(), next = profiles.begin();
        curr++;
        next++;
        if (next != profiles.end())next++;

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
                type_t leftReduction2, rightReduction2;
                if (rightSide)
                {
                    leftReduction2 = preProfile.offsetx2 + currProfile.offsetx2;
                    rightReduction2 = (-preProfile.offsetx2 + preProfile.laneCount * 2) -
                        (-currProfile.offsetx2 + currProfile.laneCount * 2);
                }
                else
                {
                    leftReduction2 = currProfile.offsetx2 - preProfile.offsetx2;
                    rightReduction2 = (preProfile.offsetx2 + preProfile.laneCount * 2) -
                        (currProfile.offsetx2 + currProfile.laneCount * 2);
                }
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
                type_t leftExpansion2, rightExpansion2;
                if (rightSide)
                {
                    leftExpansion2 = currProfile.offsetx2 - preProfile.offsetx2;
                    rightExpansion2 = (-currProfile.offsetx2 + currProfile.laneCount * 2) -
                        (-preProfile.offsetx2 + preProfile.laneCount * 2);
                }
                else
                {
                    leftExpansion2 = preProfile.offsetx2 - currProfile.offsetx2;
                    rightExpansion2 = (currProfile.offsetx2 + currProfile.laneCount * 2) - 
                        (preProfile.offsetx2 + preProfile.laneCount * 2);
                }
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
            // Lane offset
            if (transition.oldCenter2 != transition.newCenter2)
            {
                for (auto laneOffsetTransition : _MakeTransition(
                    tranS, straightS,
                    transition.oldCenter2, 
                    transition.newCenter2))
                {
                    laneOffsetResult.insert(laneOffsetTransition);
                }
            }
            if (transition.oldCenter2 != transition.newCenter2 || i == 0)
            {
                odr::Poly3 straight(
                    to_odr_unit(straightS),
                    to_odr_unit(transition.newCenter2),
                    0, 0, 0
                );
                laneOffsetResult.emplace(to_odr_unit(straightS), straight);
            }

            // Lane section
            if (rightSide)
            SPDLOG_INFO("Right transition {}: s={}, {} | {} | {}",
                i, transition.cumulativeS, transition.newLanesOnLeft, transition.startLanes, transition.newLanesOnRight);

            if (transition.newLanesOnLeft != 0 || transition.newLanesOnRight != 0)
            {
                {
                    // varying section
                    uint32_t laneIndex = 0;
                    odr::LaneSection transitionSection(roadID, to_odr_unit(tranS));
                    odr::Lane center(roadID, to_odr_unit(tranS), laneIDMultiplier * laneIndex++, false, "");
                    transitionSection.id_to_lane.emplace(center.id, center);

                    for (int i = 0; i != std::abs(transition.newLanesOnLeft); ++i)
                    {
                        odr::Lane leftVarying(roadID, to_odr_unit(tranS), laneIDMultiplier * laneIndex++, false, "driving");

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
                        odr::Lane nonVarying(roadID, to_odr_unit(tranS), laneIDMultiplier * laneIndex++, false, "driving");
                        nonVarying.lane_width.s0_to_poly.emplace(to_odr_unit(tranS), nonVaryingWidth);
                        transitionSection.id_to_lane.emplace(nonVarying.id, nonVarying);
                    }

                    for (int i = 0; i != std::abs(transition.newLanesOnRight); ++i)
                    {
                        odr::Lane rightVarying(roadID, to_odr_unit(tranS), laneIDMultiplier * laneIndex++, false, "driving");
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
                odr::Lane center(roadID, to_odr_unit(straightS), laneIDMultiplier * laneIndex++, false, "");
                straightSection.id_to_lane.emplace(center.id, center);

                odr::Poly3 constWidth(to_odr_unit(straightS), LaneWidth, 0, 0, 0);
                for (int i = 0; i < transition.startLanes + transition.newLanesOnLeft + transition.newLanesOnRight; ++i)
                {
                    odr::Lane nonVarying(roadID, to_odr_unit(straightS), laneIDMultiplier * laneIndex++, false, "driving");
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

        std::map<double, odr::LaneSection> leftSections, rightSections;
        std::map<double, odr::Poly3> leftOffsets, rightOffsets;
        std::map<double, odr::Poly3> centerWidths;

        ConvertSide(true, rightSections, rightOffsets);
        ConvertSide(false, leftSections, leftOffsets);

        // Convert difference in L/R lane offset into median center lane
        {
            auto leftKeys = odr::get_map_keys_sorted(leftOffsets);
            auto rightKeys = odr::get_map_keys_sorted(rightOffsets);
            int leftIndex = 0, rightIndex = 0;

            while (leftIndex < leftKeys.size() || rightIndex < rightKeys.size())
            {
                double nextLeft = leftIndex + 1 == leftKeys.size() ? rtnLength : leftKeys[leftIndex + 1];
                double nextRight = rightIndex + 1 == rightKeys.size() ? rtnLength : rightKeys[rightIndex + 1];

                double keyLeft = leftKeys[leftIndex];
                double keyRight = rightKeys[rightIndex];
                double sectionStart = std::max(keyLeft, keyRight);
                double sectionEnd = std::min(nextLeft, nextRight);

                const odr::Poly3& leftOffset = leftOffsets.at(keyLeft);
                const odr::Poly3& rightOffset = rightOffsets.at(keyRight);
                odr::Poly3 median;
                median.a = leftOffset.a - rightOffset.a;
                median.b = leftOffset.b - rightOffset.b;
                median.c = leftOffset.c - rightOffset.c;
                median.d = leftOffset.d - rightOffset.d;

                if (centerWidths.empty() ||
                    std::abs(centerWidths.rbegin()->second.a - median.a) +
                    std::abs(centerWidths.rbegin()->second.b - median.b) +
                    std::abs(centerWidths.rbegin()->second.c - median.c) +
                    std::abs(centerWidths.rbegin()->second.d - median.d) > 1e-3)
                {
                    centerWidths.emplace(sectionStart, median);
                    SPDLOG_INFO("Merged Center: L=({}, {}), R=({}, {})", keyLeft, nextLeft, keyRight, nextRight);
                    SPDLOG_INFO("    Width at {}: a={};b={};c={};d={}\n from l:a={};b={};c={};d={}\n from r:a={};b={};c={};d={}",
                        sectionStart, median.a, median.b, median.c, median.d
                    , leftOffset.a, leftOffset.b, leftOffset.c, leftOffset.d,
                        rightOffset.a, rightOffset.b, rightOffset.c, rightOffset.d);
                }

                if (sectionEnd == nextRight)
                {
                    rightIndex++;
                }
                if (sectionEnd == nextLeft)
                {
                    leftIndex++;
                }
            }
        }

        // Merge LaneSections
        {
            auto leftKeys = odr::get_map_keys_sorted(leftSections);
            auto centerKeys = odr::get_map_keys_sorted(centerWidths);
            auto rightKeys = odr::get_map_keys_sorted(rightSections);
            int leftIndex = 0, centerIndex = 0, rightIndex = 0;
            while (leftIndex < leftKeys.size() || centerIndex < centerKeys.size() || rightIndex < rightKeys.size())
            {
                double nextLeft = leftIndex + 1 == leftKeys.size() ? rtnLength : leftKeys[leftIndex + 1];
                double nextCenter = centerIndex + 1 == centerKeys.size() ? rtnLength : centerKeys[centerIndex + 1];
                double nextRight = rightIndex + 1 == rightKeys.size() ? rtnLength : rightKeys[rightIndex + 1];

                double keyLeft = leftKeys[leftIndex];
                double keyCenter = centerKeys[centerIndex];
                double keyRight = rightKeys[rightIndex];
                double sectionStart = std::max({ keyLeft, keyCenter, keyRight });
                double sectionEnd = std::min({ nextLeft, nextCenter, nextRight });

                SPDLOG_INFO("Merged LaneSection:({}, {}) From: L=({}, {}), C=({}, {}) R=({}, {})",
                    sectionStart, sectionEnd,
                    keyLeft, nextLeft,
                    keyCenter, nextCenter,
                    keyRight, nextRight);

                const odr::LaneSection& leftSection = leftSections.at(keyLeft);
                const odr::LaneSection& rightSection = rightSections.at(keyRight);
                odr::Poly3 centerWidth = centerWidths[keyCenter];

                odr::LaneSection section(roadID, sectionStart);
                odr::Lane center(roadID, sectionStart, 0, false, "");
                section.id_to_lane.emplace(0, center);

                for (const auto& idToLane : rightSection.id_to_lane)
                {
                    odr::Lane newLane(roadID, sectionStart, idToLane.first, false, "driving");
                    newLane.lane_width.s0_to_poly = idToLane.second.lane_width.s0_to_poly;
                    section.id_to_lane.emplace(idToLane.first, newLane);
                }

                int leftIDStart = 0;
                if (std::abs(centerWidth.a) + std::abs(centerWidth.b) + std::abs(centerWidth.c) + std::abs(centerWidth.d) > 1e-3)
                {
                    centerWidth.ComputeRelative(sectionStart);
                    SPDLOG_INFO("Center width at s = {}: a={} d={}", keyCenter, centerWidth.raw_a, centerWidth.raw_d);
                    odr::Lane medianLane(roadID, sectionStart, -1, false, "median");
                    medianLane.lane_width.s0_to_poly.emplace(sectionStart, centerWidth);
                    section.id_to_lane.emplace(1, medianLane);

                    leftIDStart = 1;
                }

                for (const auto& idToLane : leftSection.id_to_lane)
                {
                    odr::Lane newLane(roadID, sectionStart, idToLane.first, false, "driving");
                    newLane.lane_width.s0_to_poly = idToLane.second.lane_width.s0_to_poly;
                    section.id_to_lane.emplace(idToLane.first + leftIDStart, newLane);
                }

                rtn.s_to_lanesection.emplace(sectionStart, section);

                if (sectionEnd == nextLeft)
                {
                    leftIndex++;
                }
                if (sectionEnd == nextCenter)
                {
                    centerIndex++;
                }
                if (sectionEnd == nextRight)
                {
                    rightIndex++;
                }
            }
        }

        rtn.lane_offset.s0_to_poly = rightOffsets;
        return rtn;
    } // class function
} // namespace