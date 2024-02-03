#include "road.h"

#include <algorithm>
#include <vector>
#include <sstream>

#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"

#include "spdlog/spdlog.h"

namespace RoadRunner
{
    double to_odr_unit(type_s l) { return (double)l / 100; }

    double to_odr_unit(type_t l) { return (double)l / 2 * LaneWidth; }

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

    RoadProfile Road::LeftEntrance() const 
    {
        return leftProfiles.empty() ? RoadProfile{ 0, 0 } : leftProfiles.rbegin()->profile;
    }

    RoadProfile Road::LeftExit() const
    {
        return leftProfiles.empty() ? RoadProfile{ 0, 0 } : leftProfiles.begin()->profile;
    }

    RoadProfile Road::RightEntrance() const
    {
        return rightProfiles.empty() ? RoadProfile{ 0, 0 } : rightProfiles.begin()->profile;
    }

    RoadProfile Road::RightExit() const
    {
        return rightProfiles.empty() ? RoadProfile{ 0, 0 } : rightProfiles.rbegin()->profile;
    }

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
        std::list<LaneSection> profiles = rightSide ? rightProfiles : leftProfiles;
        if (!rightSide)
        {
            // Use uniform s that follows traffic direction
            for (auto& section : profiles)
            {
                section.s = Length() - section.s;
            }
        }

        /*
        Prepare transitionInfo
        */
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
            spdlog::trace("PreS {} CurrS {} NextS {}", preSection.s, currSection.s, (next == profiles.cend() ? Length() : next->s));
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

        /*
        Make odr::Road::LaneOffset & odr::LaneSection out of TransitionInfo
        */
        // Temporarily store zero-length straight section, to assist computing lane linkage between transition sections
        std::shared_ptr<odr::LaneSection> vanishedStraight = nullptr;
        for (int i = 0; i != transitions.size(); ++i)
        {
            const TransitionInfo& transition = transitions[i];
            type_s tranS = transition.cumulativeS - transition.transitionHalfLength;
            type_s straightS = transition.cumulativeS + transition.transitionHalfLength;
            type_s nextTranS = i == transitions.size() - 1 ? Length() :
                transitions[i + 1].cumulativeS - transitions[i + 1].transitionHalfLength;
            spdlog::trace("In {} Transition {}-{}-{}:", rightSide ? "Right" : "Left", tranS, straightS, nextTranS);
            spdlog::trace("L+={} | Lanes={} | R+={}", transition.newLanesOnLeft, transition.startLanes, transition.newLanesOnRight);

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

            if (straightS != nextTranS)
            {
                for (auto st : _MakeStraight(straightS, nextTranS, transition.newCenter2, rightSide))
                {
                    laneOffsetResult[st.first] = st.second;
                }
            }

            // Lane section

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

                double prevS = rightSide ? laneSectionResult.rbegin()->first : laneSectionResult.begin()->first;
                odr::LaneSection prevSection = laneSectionResult.at(prevS);

                if (vanishedStraight.get() != nullptr)
                {
                    assert(vanishedStraight->id_to_lane.size() <= transitionSection.id_to_lane.size());
                    for (int laneID = 1; laneID < vanishedStraight->id_to_lane.size(); ++laneID)
                    {
                        int vanishedLaneNext = laneIDMultiplier * (laneID + std::max(0, transition.newLanesOnLeft));
                        int vanishedLanePrev = vanishedStraight->id_to_lane.at(laneIDMultiplier * laneID).predecessor;
                        odr::Lane prevLane = prevSection.id_to_lane.at(vanishedLanePrev);
                        prevLane.successor = vanishedLaneNext;

                        odr::Lane& successorLane = transitionSection.id_to_lane.at(prevLane.successor);
                        successorLane.predecessor = vanishedLanePrev;

                        laneSectionResult.at(prevS).id_to_lane.erase(vanishedLanePrev);
                        laneSectionResult.at(prevS).id_to_lane.emplace(vanishedLanePrev, prevLane);

                        spdlog::trace(" Write succ at s={} vanished {}",
                            prevS, 
                            laneSectionResult.at(prevS).id_to_lane.at(vanishedLanePrev).successor);
                    }
                    vanishedStraight = nullptr;
                }
                else
                {
                    assert(prevSection.id_to_lane.size() <= transitionSection.id_to_lane.size());
                    for (int laneID = 1; laneID < prevSection.id_to_lane.size(); ++laneID)
                    {
                        odr::Lane preLane = prevSection.id_to_lane.at(laneIDMultiplier * laneID);

                        preLane.successor = laneIDMultiplier * (laneID + std::max(0, transition.newLanesOnLeft));

                        odr::Lane& successorLane = transitionSection.id_to_lane.at(preLane.successor);
                        successorLane.predecessor = laneIDMultiplier * laneID;

                        laneSectionResult.at(prevS).id_to_lane.erase(laneIDMultiplier * laneID);
                        laneSectionResult.at(prevS).id_to_lane.emplace(laneIDMultiplier * laneID, preLane);
                    }
                }

                laneSectionResult[tran_s_odr] = transitionSection;
            }
            
            {
                // constant section
                auto constWidth = _MakeStraight(straightS, nextTranS, 2, rightSide);
                double straight_s_odr = constWidth.begin()->first;

                uint32_t laneIndex = 0;
                assert(vanishedStraight.get() == nullptr);
                vanishedStraight = std::make_shared<odr::LaneSection>(roadID, straight_s_odr);
                odr::Lane center(roadID, straight_s_odr, laneIDMultiplier * laneIndex++, false, "");
                vanishedStraight->id_to_lane.emplace(center.id, center);

                for (int i = 0; i < transition.startLanes + transition.newLanesOnLeft + transition.newLanesOnRight; ++i)
                {
                    odr::Lane nonVarying(roadID, straight_s_odr, laneIDMultiplier * laneIndex++, false, "driving");
                    for (auto sr : constWidth)
                    {
                        nonVarying.lane_width.s0_to_poly.insert(sr);
                    }
                    vanishedStraight->id_to_lane.emplace(nonVarying.id, nonVarying);
                }
                
                if (!laneSectionResult.empty())
                {
                    double prevS = rightSide ? laneSectionResult.rbegin()->first : laneSectionResult.begin()->first;
                    auto prevSection = laneSectionResult.at(prevS);
                    assert(prevSection.id_to_lane.size() >= vanishedStraight->id_to_lane.size());

                    for (int laneID = 1; laneID < vanishedStraight->id_to_lane.size(); ++laneID)
                    {
                        odr::Lane& nextLane = vanishedStraight->id_to_lane.at(laneIDMultiplier * laneID);
                        nextLane.predecessor = laneIDMultiplier * (laneID + std::max(0, -transition.newLanesOnLeft));

                        if (straightS != nextTranS)
                        {
                            odr::Lane& predecessorLane = prevSection.id_to_lane.at(nextLane.predecessor);
                            predecessorLane.successor = laneIDMultiplier * laneID;
                        
                            laneSectionResult.at(prevS).id_to_lane.erase(nextLane.predecessor);
                            laneSectionResult.at(prevS).id_to_lane.emplace(nextLane.predecessor, predecessorLane);
                        }
                    }
                }
                
                if (straightS != nextTranS)
                {
                    laneSectionResult[straight_s_odr] = *vanishedStraight;
                    vanishedStraight = nullptr;
                }
                else
                {
                    spdlog::trace("Generate vanishedStraight");
                }
            }
        }
    }

    std::map<double, odr::Poly3> Road::_ComputeMedian(
        const std::map<double, odr::Poly3>& leftOffsets,
        const std::map<double, odr::Poly3> rightOffsets) const
    {
        const double rtnLength = to_odr_unit(Length());
        std::map<double, odr::Poly3> centerWidths;

        auto leftKeys = odr::get_map_keys_sorted(leftOffsets);
        auto rightKeys = odr::get_map_keys_sorted(rightOffsets);
        int leftIndex = 0, rightIndex = 0;

        while (leftIndex + 1 <= leftKeys.size() || rightIndex + 1 <= rightKeys.size())
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
                spdlog::trace("Merged Center: L=({}, {}), R=({}, {})", keyLeft, nextLeft, keyRight, nextRight);
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
        return centerWidths;
    }

    void Road::_MergeSides(odr::Road& rtn,
        const std::map<double, odr::LaneSection>& leftSections,
        const std::map<double, odr::Poly3>& centerWidths,
        const std::map<double, odr::LaneSection>& rightSections) const
    {
        double rtnLength = to_odr_unit(Length());

        auto leftKeys = odr::get_map_keys_sorted(leftSections);
        auto centerKeys = odr::get_map_keys_sorted(centerWidths);
        auto rightKeys = odr::get_map_keys_sorted(rightSections);
        int leftIndex = 0, centerIndex = 0, rightIndex = 0;
        while (leftIndex + 1 <= leftKeys.size() || centerIndex + 1 <= centerKeys.size() || rightIndex + 1 <= rightKeys.size())
        {
            double nextLeft = leftIndex + 1 == leftKeys.size() ? rtnLength : leftKeys[leftIndex + 1];
            double nextCenter = centerIndex + 1 == centerKeys.size() ? rtnLength : centerKeys[centerIndex + 1];
            double nextRight = rightIndex + 1 == rightKeys.size() ? rtnLength : rightKeys[rightIndex + 1];

            double keyLeft = leftKeys[leftIndex];
            double keyCenter = centerKeys[centerIndex];
            double keyRight = rightKeys[rightIndex];
            double sectionStart = std::max({ keyLeft, keyCenter, keyRight });
            double sectionEnd = std::min({ nextLeft, nextCenter, nextRight });

            spdlog::trace("Merged LaneSection:({}, {}) From: L=({}, {}), C=({}, {}) R=({}, {})",
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
                const odr::Lane& rightLane = idToLane.second;
                int newLaneID = idToLane.first;
                if (newLaneID == 0) continue; // Skip center lane

                odr::Lane newLane(roadID, sectionStart, newLaneID, false, "driving");
                for (auto s0_poly : rightLane.lane_width.s0_to_poly)
                {
                    s0_poly.second.ComputeRelative(sectionStart);
                    newLane.lane_width.s0_to_poly.emplace(s0_poly.first - keyRight + sectionStart, s0_poly.second);
                    if (sectionStart != 0)
                    {
                        if (sectionStart == keyRight)
                        {
                            newLane.predecessor = rightLane.predecessor;
                        }
                        else
                        {
                            newLane.predecessor = newLaneID; // Identical
                        }
                    }
                    if (sectionEnd != rtnLength)
                    {
                        if (sectionEnd == nextRight)
                        {
                            newLane.successor = rightLane.successor;

                        }
                        else
                        {
                            newLane.successor = newLaneID; // Identical
                        }
                    }
                }
                section.id_to_lane.emplace(newLaneID, newLane);
            }

            const int leftIDStart = 1;
            
            {
                centerWidth.ComputeRelative(sectionStart);
                odr::Lane medianLane(roadID, sectionStart, leftIDStart, false, "median");
                if (std::abs(centerWidth.a) + std::abs(centerWidth.b) + std::abs(centerWidth.c) + std::abs(centerWidth.d) > 1e-3)
                    medianLane.lane_width.s0_to_poly.emplace(keyCenter, centerWidth);
                section.id_to_lane.emplace(1, medianLane);
            }

            for (const auto& idToLane : leftSection.id_to_lane)
            {
                const odr::Lane& leftLane = idToLane.second;
                int newLaneID = idToLane.first + leftIDStart;
                if (newLaneID == 1) continue; // Skip center lane

                odr::Lane newLane(roadID, sectionStart, newLaneID, false, "driving");
                for (auto s0_poly : leftLane.lane_width.s0_to_poly)
                {
                    s0_poly.second.ComputeRelative(sectionStart);
                    newLane.lane_width.s0_to_poly.emplace(s0_poly.first - keyLeft + sectionStart, s0_poly.second);
                }

                if (sectionEnd != rtnLength)
                {
                    if (sectionEnd == nextLeft)
                    {
                        if (leftLane.predecessor != 0)
                        {
                            newLane.predecessor = leftLane.predecessor + leftIDStart;

                        }
                    }
                    else
                    {
                        newLane.predecessor = newLaneID; // Identical
                    }
                }

                if (sectionStart != 0)
                {
                    if (sectionStart == keyLeft)
                    {
                        if (leftLane.successor != 0)
                        {
                            newLane.successor = leftLane.successor + leftIDStart;
                        }
                    }
                    else
                    {
                        newLane.successor = newLaneID;  // Identical
                    }
                }

                section.id_to_lane.emplace(newLaneID, newLane);
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

    Road::operator odr::Road() const
    {
        // Fail if either side is undefined
        assert(Length() > 0);
        assert(!leftProfiles.empty() || !rightProfiles.empty());

        double rtnLength = to_odr_unit(Length());
        odr::Road rtn(roadID, rtnLength, "-1", "road " + roadID);
        rtn.ref_line.s0_to_geometry[0] = std::make_unique<odr::Line>(0, 0, 0, 0, rtnLength);

        std::map<double, odr::LaneSection> leftSections, rightSections;
        std::map<double, odr::Poly3> leftOffsets, rightOffsets;

        if (!rightProfiles.empty())
        {
            ConvertSide(true, rightSections, rightOffsets);
        }

        if (!leftProfiles.empty())
        {
            ConvertSide(false, leftSections, leftOffsets);
        }
        // from this point, s keys align with road coordinate

        // Special cases: single-direction road
        if (rightProfiles.empty())
        {
            rtn.lane_offset.s0_to_poly = leftOffsets;
            rtn.s_to_lanesection = leftSections;
            return rtn;
        }
        if (leftProfiles.empty())
        {
            rtn.lane_offset.s0_to_poly = rightOffsets;
            rtn.s_to_lanesection = rightSections;
            return rtn;
        }

        // General case
        rtn.lane_offset.s0_to_poly = rightOffsets;

        std::stringstream SPDLOG_LKEYS, SPDLOG_RKEYS;
        std::for_each(rightSections.cbegin(), rightSections.cend(),
            [&SPDLOG_RKEYS](auto s) {SPDLOG_RKEYS << s.first << " "; });

        std::for_each(leftSections.cbegin(), leftSections.cend(),
            [&SPDLOG_LKEYS](auto s) {SPDLOG_LKEYS << s.first << " "; });
        
        spdlog::trace("Right Keys: {}", SPDLOG_RKEYS.str());
        spdlog::trace("Left Keys:  {}", SPDLOG_LKEYS.str());

        std::map<double, odr::Poly3> centerWidths = _ComputeMedian(leftOffsets, rightOffsets);

        std::stringstream SPDLOG_CKEYS;
        std::for_each(centerWidths.cbegin(), centerWidths.cend(),
            [&SPDLOG_CKEYS](auto s) {SPDLOG_CKEYS << s.first << " "; });
        spdlog::trace("Center Keys:  {}", SPDLOG_CKEYS.str());

        _MergeSides(rtn, leftSections, centerWidths, rightSections);

        return rtn;
    } // class function
} // namespace