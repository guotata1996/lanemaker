#include "road_profile.h"
#include "Road.h"

#include <sstream>

#include "spdlog/spdlog.h"

namespace RoadRunner
{
    double to_odr_unit(type_s l) { return (double)l / 100; }

    double to_odr_unit(type_t l) { return (double)l / 2 * LaneWidth; }

    double to_odr_unit(double) = delete;

    type_s from_odr_unit(double l) { return std::floor(l * 100); }

    type_s from_odr_unit(type_s) = delete;

    LaneProfile::LaneProfile(uint8_t nLanes_Left, int8_t offsetX2_Left, uint8_t nLanes_Right, int8_t offsetX2_Right)
    { 
        if (nLanes_Right != 0)
        {
            rightPlan.emplace(0, LanePlan{offsetX2_Right, static_cast<int8_t>(nLanes_Right)});
        }
        if (nLanes_Left != 0)
        {
            leftPlans.emplace(std::numeric_limits<uint32_t>::max(), LanePlan{offsetX2_Left, static_cast<int8_t>(nLanes_Left)});
        }
    }

    LaneProfile& LaneProfile::operator=(const LaneProfile& other)
    {
        leftPlans = other.leftPlans;
        rightPlan = other.rightPlan;
        return *this;
    }

    void LaneProfile::RemoveRedundantProfileKeys(int side)
    {
        std::set<type_s> toRemove;
        if (side < 0 && rightPlan.size() >= 2)
        {
            for (auto it = rightPlan.begin(); ; ++it)
            {
                auto end = it;
                end++;
                if (end == rightPlan.end()) break;
                if (it->second == end->second)
                {
                    toRemove.insert(end->first);
                }
            }

            for (type_s s : toRemove)
            {
                rightPlan.erase(s);
            }
        }
        else if (side > 0 && leftPlans.size() >= 2)
        {
            for (auto it = leftPlans.rbegin(); ; ++it)
            {
                auto end = it;
                end++;
                if (end == leftPlans.rend()) break;
                if (it->second == end->second)
                {
                    toRemove.insert(end->first);
                }
            }

            for (type_s s : toRemove)
            {
                leftPlans.erase(s);
            }
        }
    }

    void LaneProfile::OverwriteSection(int side, type_s start, type_s end, uint8_t nLanes, int8_t offsetX2)
    {
        if (side < 0)
        {
            if (start >= end)
                throw std::logic_error("OverwriteSection: param error");
        }
        else
        {
            if (start <= end)
                throw std::logic_error("OverwriteSection: param error");
        }

        auto& profileToModify = side > 0 ? leftPlans : rightPlan;
        if (profileToModify.empty()) 
        {
            throw std::logic_error("Override target side has empty profile!");
        }
        auto existingKeys = odr::get_map_keys_sorted(profileToModify);

        type_s largestElementEqualOrBefore;
        if (side < 0)
        {
            for (auto it = existingKeys.rbegin(); it != existingKeys.rend(); ++it)
            {
                largestElementEqualOrBefore = *it;
                if (largestElementEqualOrBefore <= end) break;
            }
        }
        else
        {
            largestElementEqualOrBefore = profileToModify.lower_bound(end)->first;
        }

        auto existingProfileAtEnd = profileToModify.at(largestElementEqualOrBefore);

        // Do overwrite
        profileToModify[start] = LanePlan{ offsetX2, static_cast<int8_t>(nLanes) };
        for (auto it = existingKeys.rbegin(); it != existingKeys.rend(); ++it)
        {
            if (start < *it && *it < end || end < *it && *it < start)
            {
                profileToModify.erase(*it);
            }
        }
        profileToModify[end] = existingProfileAtEnd;

        RemoveRedundantProfileKeys(side);

        spdlog::trace("====side {} ===", side);
        for (auto s_section : profileToModify) {
            spdlog::trace("s = {}: nLanes{} offset{}",
                s_section.first, s_section.second.laneCount, s_section.second.offsetx2);
        }
    }

    LanePlan LaneProfile::LeftEntrance() const
    {
        switch (leftPlans.size())
        {
        case 0:
            return LanePlan{ 0, 0 };
        case 1:
            return leftPlans.rbegin()->second;
        default:
            auto maxKey = leftPlans.rbegin();
            maxKey++;
            return maxKey->second;
            break;
        }
    }

    LanePlan LaneProfile::LeftExit() const
    {
        return leftPlans.empty() ? LanePlan{ 0, 0 } : leftPlans.begin()->second;
    }

    LanePlan LaneProfile::RightEntrance() const
    {
        return rightPlan.empty() ? LanePlan{ 0, 0 } : rightPlan.begin()->second;
    }

    LanePlan LaneProfile::RightExit() const
    {
        return rightPlan.empty() ? LanePlan{ 0, 0 } : rightPlan.rbegin()->second;
    }

    std::map<std::pair<type_s, type_s>, LanePlan> LaneProfile::GetAllSections(type_s length, int side) const
    {
        std::map<std::pair<type_s, type_s>, LanePlan> rtn;
        if (side == -1)
        {
            if (rightPlan.empty()) return rtn;
            auto rightKeys = odr::get_map_keys_sorted(rightPlan);
            if (std::find(rightKeys.begin(), rightKeys.end(), length) == rightKeys.end())
            {
                rightKeys.push_back(length);
                std::sort(rightKeys.begin(), rightKeys.end());
            }
            for (int i = 0; i < rightKeys.size() - 1 && rightKeys[i] < length; ++i)
            {
                int j = i + 1;
                auto section = rightPlan.at(rightKeys[i]);

                type_s overwriteStart = rightKeys[i];
                type_s overwriteEnd = std::min(rightKeys[j], length);
                rtn.emplace(std::make_pair(overwriteStart, overwriteEnd), section);
            }
        }
        else
        {
            if (leftPlans.empty()) return rtn;
            auto oldLeftKeys = odr::get_map_keys_sorted(leftPlans);
            if (std::find(oldLeftKeys.begin(), oldLeftKeys.end(), 0) == oldLeftKeys.end())
            {
                oldLeftKeys.insert(oldLeftKeys.begin(), 0);
                std::sort(oldLeftKeys.begin(), oldLeftKeys.end());
            }
            for (int i = 0; i < oldLeftKeys.size() - 1 && oldLeftKeys[i] < length; ++i)
            {
                int j = i + 1;
                auto section = leftPlans.at(oldLeftKeys[j]);

                type_s overwriteStart = std::min(oldLeftKeys[j], length);
                type_s overwriteEnd = oldLeftKeys[i];
                rtn.emplace(std::make_pair(overwriteStart, overwriteEnd), section);
            }
        }
        return rtn;
    }

    std::set<type_s> LaneProfile::GetAllKeys(type_s length) const
    {
        std::set<type_s> rtn{0, length };
        auto rightKeys = odr::get_map_keys(rightPlan);
        for (auto key : rightKeys)
        {
            if (key < length)
            {
                rtn.insert(key);
            }
        }
        auto leftKeys = odr::get_map_keys(leftPlans);
        for (auto key : leftKeys)
        {
            if (key < length)
            {
                rtn.insert(key);
            }
        }
        return rtn;
    }

    bool LaneProfile::HasSide(int side) const
    {
        return side < 0 ? !rightPlan.empty() : !leftPlans.empty();
    }

    std::map<double, odr::Poly3> LaneProfile::_MakeTransition(
        type_s start_s, type_s end_s,
        type_t start_t2, type_t end_t2, bool rightSide, type_s length) const
    {
        if (start_s >= end_s)
            throw std::logic_error("MakeTransition param error!");

        double odr_start_s = to_odr_unit(start_s);
        if (!rightSide)
        {
            odr_start_s = to_odr_unit(length - end_s);
            std::swap(start_s, end_s);
            start_s = length - start_s;
            end_s = length - end_s;
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

    std::map<double, odr::Poly3> LaneProfile::_MakeStraight(type_s start_s, type_s end_s, type_t const_t,
        bool rightSide, type_s length) const
    {
        if (start_s > end_s)
            throw std::logic_error("MakeStraight param error!");
        double odr_start_s = rightSide ? to_odr_unit(start_s) : to_odr_unit(length - end_s);

        return { std::make_pair(
            odr_start_s,
            odr::Poly3(odr_start_s, to_odr_unit(const_t), 0, 0, 0))
        };
    }

    void LaneProfile::ConvertSide(bool rightSide, std::string roadID, type_s length,
        std::map<double, odr::LaneSection>& laneSectionResult,
        std::map<double, odr::Poly3>& laneOffsetResult) const
    {
        decltype(rightPlan) profiles;

        if (rightSide)
        {
            for (const auto& s_and_section : rightPlan)
            {
                type_s uniformS = s_and_section.first;
                if (uniformS < ProfileMinLengthCM)
                {
                    uniformS = 0;
                }
                if (uniformS < length - ProfileMinLengthCM)
                {
                    profiles.emplace(uniformS, s_and_section.second);
                }
            }
        }
        else
        {
            // Use uniform s that follows traffic direction
            for (const auto& s_and_section : leftPlans)
            {
                type_s clampedS = std::min(s_and_section.first, length); // filter out uint32_max value
                type_s uniformS = length - clampedS;
                if (uniformS < ProfileMinLengthCM)
                {
                    uniformS = 0;
                }
                if (uniformS < length - ProfileMinLengthCM)
                {
                    // Ignore impossible transition at the end
                    profiles.emplace(uniformS, s_and_section.second);
                }
            }
        }

        /*
        Prepare transitionInfo
        */
        std::map<type_s, LanePlan>::const_iterator pre = profiles.begin(), curr = profiles.begin();
        curr++;

        std::vector<TransitionInfo> transitions(profiles.size() + 1);
        transitions[0] = TransitionInfo
        {
            0,
            profiles.begin()->second.offsetx2, profiles.begin()->second.offsetx2,
            profiles.begin()->second.laneCount, 0, 0,
            0
        };  // first element is dummy
        int transitionIndex = 1;
        for (; curr != profiles.end(); ++pre, ++curr, ++transitionIndex)
        {
            type_s preSectionS = pre->first;
            type_s currSectionS = curr->first;
            const LanePlan& preProfile = pre->second;
            const LanePlan& currProfile = curr->second;
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

            auto next = curr;
            next++;

            type_s preLength = currSectionS - preSectionS;
            type_s nextLength = (next == profiles.cend() ? length : next->first) - currSectionS;
            spdlog::trace("PreS {} CurrS {} NextS {}", preSectionS, currSectionS, (next == profiles.cend() ? length : next->first));
            type_s preTransitionLength = std::min({preLength / 2, nextLength / 2, MaxTransitionS});
            if (preTransitionLength == 0)
                throw std::logic_error("Transition cannot have zero length!");

            // write
            transitions[transitionIndex] = TransitionInfo
            {
                currSectionS,
                preProfile.offsetx2, currProfile.offsetx2,
                preProfile.laneCount, newLanesOnLeft , newLanesOnRight,
                preTransitionLength
            };
        }
        // last transition is dummy
        transitions[profiles.size()] = TransitionInfo
        {
            length,
            profiles.rbegin()->second.offsetx2, profiles.rbegin()->second.offsetx2,
            profiles.rbegin()->second.laneCount, 0, 0,
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
            type_s nextTranS = i == transitions.size() - 1 ? length :
                transitions[i + 1].cumulativeS - transitions[i + 1].transitionHalfLength;
            spdlog::trace("In {} Transition {}-{}-{}:", rightSide ? "Right" : "Left", tranS, straightS, nextTranS);
            spdlog::trace("L+={} | Lanes={} | R+={}", transition.newLanesOnLeft, transition.startLanes, transition.newLanesOnRight);

            // Lane offset
            // Transition MUST NOT happen at 0 or L
            if (transition.cumulativeS != 0 && transition.cumulativeS != length)
            {
                for (auto t : _MakeTransition(
                    tranS, straightS,
                    transition.oldCenter2,
                    transition.newCenter2, rightSide, length))
                {
                    laneOffsetResult[t.first] = t.second;
                }
            }

            if (straightS != nextTranS)
            {
                for (auto st : _MakeStraight(straightS, nextTranS, transition.newCenter2, rightSide, length))
                {
                    laneOffsetResult[st.first] = st.second;
                }
            }

            // Lane section

            const int laneIDMultiplier = rightSide ? -1 : 1;
            if (transition.cumulativeS != 0 && transition.cumulativeS != length)
            {
                // varying section
                auto varyWidthL = _MakeTransition(
                    tranS, straightS,
                    transition.newLanesOnLeft > 0 ? 0 : 2,
                    transition.newLanesOnLeft > 0 ? 2 : 0, rightSide, length);
                auto constWidth = _MakeStraight(tranS, straightS, 2, rightSide, length);
                auto varyWidthR = _MakeTransition(
                    tranS, straightS,
                    transition.newLanesOnRight > 0 ? 0 : 2,
                    transition.newLanesOnRight > 0 ? 2 : 0, rightSide, length);
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
                    if (vanishedStraight->id_to_lane.size() > transitionSection.id_to_lane.size())
                        throw std::logic_error("LaneProfile::Convertside");
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
                    if (prevSection.id_to_lane.size() > transitionSection.id_to_lane.size())
                        throw std::logic_error("LaneProfile::Convertside");
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
                auto constWidth = _MakeStraight(straightS, nextTranS, 2, rightSide, length);
                double straight_s_odr = constWidth.begin()->first;

                uint32_t laneIndex = 0;
                if (vanishedStraight.get() != nullptr)
                    throw std::logic_error("LaneProfile::Convertside");
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
                    if (prevSection.id_to_lane.size() < vanishedStraight->id_to_lane.size())
                        throw std::logic_error("LaneProfile::Convertside");

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

    std::map<double, odr::Poly3> LaneProfile::_ComputeMedian(
        const std::map<double, odr::Poly3>& leftOffsets,
        const std::map<double, odr::Poly3> rightOffsets, type_s length) const
    {
        const double rtnLength = to_odr_unit(length);
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

    void LaneProfile::_MergeSides(odr::Road* rtn,
        const std::map<double, odr::LaneSection>& leftSections,
        const std::map<double, odr::Poly3>& centerWidths,
        const std::map<double, odr::LaneSection>& rightSections,
        type_s length) const
    {
        double rtnLength = to_odr_unit(length);

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

            odr::LaneSection section(rtn->id, sectionStart);
            odr::Lane center(rtn->id, sectionStart, 0, false, "");
            section.id_to_lane.emplace(0, center);

            for (const auto& idToLane : rightSection.id_to_lane)
            {
                const odr::Lane& rightLane = idToLane.second;
                int newLaneID = idToLane.first;
                if (newLaneID == 0) continue; // Skip center lane

                odr::Lane newLane(rtn->id, sectionStart, newLaneID, false, "driving");
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
                odr::Lane medianLane(rtn->id, sectionStart, leftIDStart, false, "median");
                if (std::abs(centerWidth.a) + std::abs(centerWidth.b) + std::abs(centerWidth.c) + std::abs(centerWidth.d) > 1e-3)
                    medianLane.lane_width.s0_to_poly.emplace(keyCenter, centerWidth);
                section.id_to_lane.emplace(1, medianLane);
            }

            for (const auto& idToLane : leftSection.id_to_lane)
            {
                const odr::Lane& leftLane = idToLane.second;
                int newLaneID = idToLane.first + leftIDStart;
                if (newLaneID == 1) continue; // Skip center lane

                odr::Lane newLane(rtn->id, sectionStart, newLaneID, false, "driving");
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

            rtn->s_to_lanesection.emplace(sectionStart, section);

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

    void LaneProfile::Apply(double _length, odr::Road* rtn)
    {
        // Fail if either side is undefined
        if (_length <= 0)
            throw std::logic_error("LaneProfile::Apply");
        if (leftPlans.empty() && rightPlan.empty()) 
        {
            throw std::logic_error("LaneProfile::Apply empty");
        }
        rtn->length = _length;

        rtn->s_to_lanesection.clear();

        type_s length = from_odr_unit(_length);

        std::map<double, odr::LaneSection> leftSections, rightSections;
        std::map<double, odr::Poly3> leftOffsets, rightOffsets;
        if (!rightPlan.empty())
        {
            // Once length is set, no key beyond length
            for (type_s key : odr::get_map_keys(rightPlan))
            {
                if (key >= length)
                {
                    rightPlan.erase(key);
                }
            }
            ConvertSide(true, rtn->id, length, rightSections, rightOffsets);
        }

        if (!leftPlans.empty())
        {
            // Once length is set, no key beyond length except max()
            auto leftKeys = odr::get_map_keys(leftPlans);
            auto trueEntryKey = *leftKeys.lower_bound(length - 2);
            if (trueEntryKey != length)
            {
                auto trueEntryProfile = leftPlans.at(trueEntryKey);
                if (trueEntryKey != std::numeric_limits<uint32_t>::max())
                {
                    leftPlans.erase(trueEntryKey);
                }
                leftPlans[length] = trueEntryProfile;
            }

            for (type_s key : leftKeys)
            {
                if (key == 0 || key > length && key != std::numeric_limits<uint32_t>::max())
                {
                    leftPlans.erase(key);
                }
            }
            
            ConvertSide(false, rtn->id, length, leftSections, leftOffsets);
        }
        // from this point, s keys align with road coordinate

        // Special cases: single-direction road
        if (rightPlan.empty())
        {
            rtn->lane_offset.s0_to_poly = leftOffsets;
            rtn->s_to_lanesection = leftSections;
            return;
        }
        if (leftPlans.empty())
        {
            rtn->lane_offset.s0_to_poly = rightOffsets;
            rtn->s_to_lanesection = rightSections;
            return;
        }

        // General case
        rtn->lane_offset.s0_to_poly = rightOffsets;

        std::stringstream SPDLOG_LKEYS, SPDLOG_RKEYS;
        std::for_each(rightSections.cbegin(), rightSections.cend(),
            [&SPDLOG_RKEYS](auto s) {SPDLOG_RKEYS << s.first << " "; });

        std::for_each(leftSections.cbegin(), leftSections.cend(),
            [&SPDLOG_LKEYS](auto s) {SPDLOG_LKEYS << s.first << " "; });

        spdlog::trace("Right Keys: {}", SPDLOG_RKEYS.str());
        spdlog::trace("Left Keys:  {}", SPDLOG_LKEYS.str());

        std::map<double, odr::Poly3> centerWidths = _ComputeMedian(leftOffsets, rightOffsets, length);

        std::stringstream SPDLOG_CKEYS;
        std::for_each(centerWidths.cbegin(), centerWidths.cend(),
            [&SPDLOG_CKEYS](auto s) {SPDLOG_CKEYS << s.first << " "; });
        spdlog::trace("Center Keys:  {}", SPDLOG_CKEYS.str());

        _MergeSides(rtn, leftSections, centerWidths, rightSections, length);
    }

    std::string LaneProfile::Log() const
    {
        std::stringstream ss;
        if (!rightPlan.empty())
        {
            ss << "\n======Right Profile======\n";
            for (auto s_profile : rightPlan)
            {
                ss << s_profile.first << " : Lanes=" << std::to_string(s_profile.second.laneCount) 
                    << " OffsetX2=" << std::to_string(s_profile.second.offsetx2) << std::endl;
            }
        }
        if (!leftPlans.empty())
        {
            ss << "\n======Left Profile======\n";
            for (auto it = leftPlans.rbegin(); it != leftPlans.rend(); ++it)
            {
                ss << it->first << " : Lanes=" << std::to_string(it->second.laneCount) 
                    << " OffsetX2=" << std::to_string(it->second.offsetx2) << std::endl;
            }
        }
        ss << "\n";
        return ss.str();
    }

    LanePlan LaneProfile::ProfileAt(double s, int side) const 
    { 
        type_s key = from_odr_unit(s);
        auto&   lookup = side < 0 ? rightPlan : leftPlans;
        if (lookup.empty()) 
        {
            return LanePlan{0, 0};
        }
        if (side < 0) 
        {
            auto it = rightPlan.upper_bound(key);
            if (it != rightPlan.cbegin())
                it--;
            return it->second;
        }
        auto   it = leftPlans.upper_bound(key);
        if (it->first == std::numeric_limits<uint32_t>::max())
            it--;
        return it->second;
    }
}