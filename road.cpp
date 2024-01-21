#include "road.h"

#include <algorithm>
#include <vector>
#include <sstream>

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
        type_t start_t2, type_t end_t2, bool rightSide) const
    {
        assert(start_s < end_s);

        double odr_start_s = to_odr_unit(start_s);
        if (!rightSide)
        {
            odr_start_s = to_odr_unit(Length() - end_s);
            std::swap(start_s, end_s);
            start_s = Length() - start_s;
            end_s = Length() - end_s;
            std::swap(start_t2, end_t2);
        }
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

        return { std::make_pair(
            odr_start_s,
            odr::Poly3(odr_start_s, a, b, c, d))
        };
    }

    std::map<double, odr::Poly3> Road::_MakeStraight(type_s start_s, type_s end_s, type_t const_t, bool rightSide) const
    {
        assert(start_s <= end_s);
        double odr_start_s = rightSide ? to_odr_unit(start_s) : to_odr_unit(Length() - end_s);

        return { std::make_pair(
            odr_start_s,
            odr::Poly3(odr_start_s, to_odr_unit(const_t), 0, 0, 0)) 
        };
    }

    void Road::ConvertSide(bool rightSide,
        std::map<double, odr::LaneSection>& laneSectionResult,
        std::map<double, odr::Poly3>& laneOffsetResult) const
    {
        const std::list<LaneSection>& profiles = rightSide ? rightProfiles : leftProfiles;

        std::list<LaneSection>::const_iterator pre = profiles.begin(), curr = profiles.begin();
        curr++;

        std::vector<TransitionInfo> transitions(profiles.size() + 1);
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
            const int TSign = rightSide ? 1 : -1;
            
            // pre->curr
            int newLanesOnLeft = 0, newLanesOnRight = 0;
            if (preProfile.laneCount > currProfile.laneCount)
            {
                // lanes merging
                int vanishedLanes = preProfile.laneCount - currProfile.laneCount;
                type_t leftReduction2, rightReduction2;
                
                leftReduction2 = TSign * preProfile.offsetx2 - TSign * currProfile.offsetx2;
                rightReduction2 = (TSign * currProfile.offsetx2 - currProfile.laneCount * 2)
                    - (TSign * preProfile.offsetx2 - preProfile.laneCount * 2);

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

                leftExpansion2 = TSign * currProfile.offsetx2 - TSign * preProfile.offsetx2;
                rightExpansion2 = (TSign * preProfile.offsetx2 - preProfile.laneCount * 2) -
                    (TSign * currProfile.offsetx2 - currProfile.laneCount * 2);

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

            std::list<LaneSection>::const_iterator next = curr;
            next++;

            type_s preLength = currSection.s - preSection.s;
            type_s nextLength = (next == profiles.cend() ? Length() : next->s) - currSection.s;
            SPDLOG_TRACE("PreS {} CurrS {} NextS {}", preSection.s, currSection.s, (next == profiles.cend() ? Length() : next->s));
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
        // last transition is dummy
        transitions[profiles.size()] = TransitionInfo
        {
            Length(),
            profiles.back().profile.offsetx2, profiles.back().profile.offsetx2,
            profiles.back().profile.laneCount, 0, 0,
            0
        };  // last transition is dummy

        for (int i = 0; i != transitions.size(); ++i)
        {
            const TransitionInfo& transition = transitions[i];
            type_s tranS = transition.cumulativeS - transition.transitionHalfLength;
            type_s straightS = transition.cumulativeS + transition.transitionHalfLength;
            type_s nextTranS = i == transitions.size() - 1 ? Length() :
                transitions[i + 1].cumulativeS - transitions[i + 1].transitionHalfLength;
            // Lane offset
            // Transition MUST NOT happen at 0 or L
            if (transition.cumulativeS != 0 && transition.cumulativeS != Length())
            {
                for (auto t : _MakeTransition(
                    tranS, straightS,
                    transition.oldCenter2, 
                    transition.newCenter2, rightSide))
                {
                    laneOffsetResult[t.first] = t.second;
                }
            }
            for (auto st : _MakeStraight(straightS, nextTranS, transition.newCenter2, rightSide))
            {
                laneOffsetResult[st.first] = st.second;
            }

            // Lane section
            SPDLOG_INFO("In {} Transition {}-{}-{}:", rightSide ? "Right" : "Left", tranS, straightS, nextTranS);
            SPDLOG_INFO("L+={} | Lanes={} | R+={}",  transition.newLanesOnLeft, transition.startLanes, transition.newLanesOnRight);

            const int laneIDMultiplier = rightSide ? -1 : 1;
            if (transition.cumulativeS != 0 && transition.cumulativeS != Length())
            {
                // varying section
                auto varyWidthL = _MakeTransition(
                    tranS, straightS,
                    transition.newLanesOnLeft > 0 ? 0 : 2,
                    transition.newLanesOnLeft > 0 ? 2 : 0, rightSide);
                auto constWidth = _MakeStraight(tranS, straightS, 2, rightSide);
                auto varyWidthR = _MakeTransition(
                    tranS, straightS,
                    transition.newLanesOnRight > 0 ? 0 : 2,
                    transition.newLanesOnRight > 0 ? 2 : 0, rightSide);
                double tran_s_odr = varyWidthL.begin()->first;

                uint32_t laneIndex = 0;
                odr::LaneSection transitionSection(roadID, tran_s_odr);
                odr::Lane center(roadID, tran_s_odr, laneIDMultiplier * laneIndex++, false, "");
                transitionSection.id_to_lane.emplace(center.id, center);

                for (int i = 0; i != std::abs(transition.newLanesOnLeft); ++i)
                {
                    odr::Lane leftVarying(roadID, tran_s_odr, laneIDMultiplier * laneIndex++, false, "driving");

                    for (auto tr : varyWidthL)
                    {
                        leftVarying.lane_width.s0_to_poly.insert(tr);
                    }
                    transitionSection.id_to_lane.emplace(leftVarying.id, leftVarying);
                }

                for (int i = 0; i != std::min(transition.startLanes,
                    transition.startLanes + transition.newLanesOnLeft + transition.newLanesOnRight);
                    ++i)
                {
                    odr::Lane nonVarying(roadID, tran_s_odr, laneIDMultiplier * laneIndex++, false, "driving");
                    for (auto tr : constWidth)
                    {
                        nonVarying.lane_width.s0_to_poly.insert(tr);
                    }
                    transitionSection.id_to_lane.emplace(nonVarying.id, nonVarying);
                }

                for (int i = 0; i != std::abs(transition.newLanesOnRight); ++i)
                {
                    odr::Lane rightVarying(roadID, tran_s_odr, laneIDMultiplier * laneIndex++, false, "driving");
                    for (auto tr : varyWidthR)
                    {
                        rightVarying.lane_width.s0_to_poly.insert(tr);
                    }
                    transitionSection.id_to_lane.emplace(rightVarying.id, rightVarying);
                }
                laneSectionResult[tran_s_odr] = transitionSection;
                    
                // TODO: calc link with prev section, if newLanesOnLeft > 0
                // else, mapping is identical
                
            }

            {
                auto constWidth = _MakeStraight(straightS, nextTranS, 2, rightSide);
                double straight_s_odr = constWidth.begin()->first;

                uint32_t laneIndex = 0;
                odr::LaneSection straightSection(roadID, straight_s_odr);
                odr::Lane center(roadID, straight_s_odr, laneIDMultiplier * laneIndex++, false, "");
                straightSection.id_to_lane.emplace(center.id, center);

                for (int i = 0; i < transition.startLanes + transition.newLanesOnLeft + transition.newLanesOnRight; ++i)
                {
                    odr::Lane nonVarying(roadID, straight_s_odr, laneIDMultiplier * laneIndex++, false, "driving");
                    for (auto sr : constWidth)
                    {
                        nonVarying.lane_width.s0_to_poly.insert(sr);
                    }
                    straightSection.id_to_lane.emplace(nonVarying.id, nonVarying);
                }
                laneSectionResult[straight_s_odr] = straightSection;
            }

            // TODO: calc link with prev section, if newLanesOnLeft < 0
            // else, mapping is identical
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
        // from this point, s keys align with road coordinate

        std::stringstream SPDLOG_LKEYS, SPDLOG_RKEYS;
        std::for_each(rightSections.cbegin(), rightSections.cend(),
            [&SPDLOG_RKEYS](auto s) {SPDLOG_RKEYS << s.first << " "; });

        std::for_each(leftSections.cbegin(), leftSections.cend(),
            [&SPDLOG_LKEYS](auto s) {SPDLOG_LKEYS << s.first << " "; });
        
        SPDLOG_INFO("Right Keys: {}", SPDLOG_RKEYS.str());
        SPDLOG_INFO("Left Keys:  {}", SPDLOG_LKEYS.str());

        // Convert difference in L/R lane offset into median center lane
        {
            auto leftKeys = odr::get_map_keys_sorted(leftOffsets);
            auto rightKeys = odr::get_map_keys_sorted(rightOffsets);
            int leftIndex = 0, rightIndex = 0;

            while (leftIndex + 1 < leftKeys.size() || rightIndex + 1 < rightKeys.size())
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
                    SPDLOG_TRACE("Merged Center: L=({}, {}), R=({}, {})", keyLeft, nextLeft, keyRight, nextRight);
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

        std::stringstream SPDLOG_CKEYS;
        std::for_each(centerWidths.cbegin(), centerWidths.cend(),
            [&SPDLOG_CKEYS](auto s) {SPDLOG_CKEYS << s.first << " "; });
        SPDLOG_INFO("Center Keys:  {}", SPDLOG_CKEYS.str());

        // Merge LaneSections
        {
            auto leftKeys = odr::get_map_keys_sorted(leftSections);
            auto centerKeys = odr::get_map_keys_sorted(centerWidths);
            auto rightKeys = odr::get_map_keys_sorted(rightSections);
            int leftIndex = 0, centerIndex = 0, rightIndex = 0;
            while (leftIndex + 1 < leftKeys.size() || centerIndex + 1 < centerKeys.size() || rightIndex + 1 < rightKeys.size())
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
                odr::Poly3 centerWidth = centerWidths.at(keyCenter);

                odr::LaneSection section(roadID, sectionStart);
                odr::Lane center(roadID, sectionStart, 0, false, "");
                section.id_to_lane.emplace(0, center);

                for (const auto& idToLane : rightSection.id_to_lane)
                {
                    odr::Lane newLane(roadID, sectionStart, idToLane.first, false, "driving");
                    for (auto s0_poly : idToLane.second.lane_width.s0_to_poly)
                    {
                        s0_poly.second.ComputeRelative(sectionStart);
                        newLane.lane_width.s0_to_poly.emplace(s0_poly.first - keyRight + sectionStart, s0_poly.second);
                    }
                    section.id_to_lane.emplace(idToLane.first, newLane);
                }

                int leftIDStart = 0;
                if (std::abs(centerWidth.a) + std::abs(centerWidth.b) + std::abs(centerWidth.c) + std::abs(centerWidth.d) > 1e-3)
                {
                    centerWidth.ComputeRelative(sectionStart);
                    odr::Lane medianLane(roadID, sectionStart, -1, false, "median");
                    medianLane.lane_width.s0_to_poly.emplace(keyCenter, centerWidth);
                    section.id_to_lane.emplace(1, medianLane);

                    leftIDStart = 1;
                }

                for (const auto& idToLane : leftSection.id_to_lane)
                {
                    odr::Lane newLane(roadID, sectionStart, idToLane.first, false, "driving");
                    for (auto s0_poly : idToLane.second.lane_width.s0_to_poly)
                    {
                        s0_poly.second.ComputeRelative(sectionStart);
                        newLane.lane_width.s0_to_poly.emplace(s0_poly.first - keyLeft + sectionStart, s0_poly.second);
                    }
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