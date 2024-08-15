#include "road_profile.h"
#include <Road.h>

#include <cassert>
/*
namespace RoadRunner
{
    ElevationProfile::ElevationProfile() 
    { 
        plans.emplace(0, 0);
    }

    void ElevationProfile::OverwriteSection(type_s start, type_s end, type_s length, ElevationPlan elevation) 
    {
        assert(0 <= start);
        assert(start < end);
        assert(end <= length);

        SnapToSegmentBoundary(start, length);
        SnapToSegmentBoundary(end, length);
        if (start >= end) 
        {
            return;
        }

        auto existingKeys = odr::get_map_keys_sorted(plans);
        type_s largestElementEqualOrBefore;
        for (auto it = existingKeys.rbegin(); it != existingKeys.rend(); ++it)
        {
            largestElementEqualOrBefore = *it;
            if (largestElementEqualOrBefore <= end)
                break;
        }
        auto existingProfileAtEnd = plans.at(largestElementEqualOrBefore);

        // Do overwrite
        plans[start] = elevation;
        for (auto it = existingKeys.rbegin(); it != existingKeys.rend(); ++it) 
        {
            if (start < *it && *it < end) 
            {
                plans.erase(*it);
            }
        }
        plans[end] = existingProfileAtEnd;

        RemoveRedundantProfileKeys();
    }

    void ElevationProfile::Apply(double _length, odr::Road* r) 
    { 
        type_s length = from_odr_unit(_length);
        auto& rtn = r->ref_line.elevation_profile;
        rtn.s0_to_poly.clear();

        auto planCopy  = plans;
        if (planCopy.find(0) == planCopy.end()) 
        {
            planCopy.emplace(0, 0);
        }
        planCopy[_length] = 0;

        for (auto it = planCopy.begin(); it != planCopy.cend(); ++it) 
        {
            auto elevation = it->second;
            double odrS = to_odr_unit(it->first);
            rtn.s0_to_poly.emplace(odrS, odr::Poly3(odrS, elevation, 0, 0, 0));
        }
    }

    ElevationPlan ElevationProfile::GetMax(type_s start, type_s end) const 
    { 
        return GetExtreme(start, end, true);
    }

    ElevationPlan ElevationProfile::GetMin(type_s start, type_s end) const 
    { 
        return GetExtreme(start, end, false);
    }

    ElevationPlan ElevationProfile::GetExtreme(type_s start, type_s end, bool _max) const 
    {
        auto start_iter = plans.upper_bound(start);
        start_iter--;
        auto end_iter = plans.lower_bound(end);
        assert(start_iter != end_iter);

        ElevationPlan rtn = _max ? - 128 : 127;
        for (auto it = start_iter; it != end_iter; ++it)
        {
            rtn = _max ? std::max(rtn, it->second) : std::min(rtn, it->second);
        }
        return rtn;
    }

    ElevationProfile ElevationProfile::Reversed(type_s length) const 
    { 
        ElevationProfile newProfile;
        for (const auto& lSectionInfo : GetAllSections(length)) 
        {
            type_s fwdStart = lSectionInfo.first.first;
            type_s fwdEnd = lSectionInfo.first.second;
            newProfile.OverwriteSection(length - fwdEnd, length - fwdStart, length, lSectionInfo.second);
        }
        return newProfile;
    }

    void ElevationProfile::Split(type_s length, type_s splitPoint, ElevationProfile& profile1, ElevationProfile& profile2) const 
    {
        SnapToSegmentBoundary(splitPoint, length);
        profile1 = ElevationProfile();
        profile2 = ElevationProfile();
        for (auto section : GetAllSections(length)) 
        {
            type_s sectionBegin = section.first.first;
            type_s sectionEnd = section.first.second;
            if (sectionEnd <= splitPoint)
            {
                profile1.OverwriteSection(sectionBegin, sectionEnd, length, section.second);
            }
            else if (sectionBegin < splitPoint && splitPoint < sectionEnd)
            {
                profile1.OverwriteSection(sectionBegin, splitPoint,   length, section.second);
                profile2.OverwriteSection(0, sectionEnd - splitPoint, length, section.second);
            }
            else if (splitPoint <= sectionEnd)
            {
                profile2.OverwriteSection(
                    sectionBegin - splitPoint, sectionEnd - splitPoint, length, section.second);
            }
        }
    }

    void ElevationProfile::Join(type_s linkBase, type_s road2Base, type_s road2Length, const ElevationProfile& road2Profile, type_s finalLength)
    {
        SnapToSegmentBoundary(linkBase, finalLength);
        SnapToSegmentBoundary(road2Base, finalLength);
        if (road2Base != linkBase) 
        {
            OverwriteSection(linkBase, road2Base, finalLength, plans.rbegin()->second);
        }
        for (const auto& road2section : road2Profile.GetAllSections(road2Length)) 
        {
            type_s newStart = road2section.first.first + road2Base;
            type_s newEnd = road2section.first.second + road2Base;
            OverwriteSection(newStart, newEnd, finalLength, road2section.second);
        }
    }

    bool ElevationProfile::SnapToSegmentBoundary(type_s& key, type_s length, type_s limit) const
    {
        if (key < limit)
        {
            key = 0;
            return true;
        }
        if (key > length - limit)
        {
            key = length;
            return true;
        }

        auto existingKeys = odr::get_map_keys(plans);

        auto above = existingKeys.lower_bound(key);
        if (*above - key < limit)
        {
            key = *above;
            return true;
        }

        if (above != existingKeys.begin())
        {
            auto below = above;
            below--;
            if (key - *below < limit)
            {
                key = *below;
                return true;
            }
        }
        return false;
    }

    std::map<std::pair<type_s, type_s>, ElevationPlan> ElevationProfile::GetAllSections(type_s length) const 
    {
        std::map<std::pair<type_s, type_s>, ElevationPlan> rtn;
        for (auto it = plans.cbegin(); it != plans.cend(); ++it) 
        {
            auto next = it;
            next++;
            type_s segEnd = next == plans.cend() ? length : next->first;
            rtn.emplace(std::make_pair(it->first, segEnd), it->second);
        }
        return rtn;
    }

    void ElevationProfile::RemoveRedundantProfileKeys() 
    { 
        std::set<type_s> toRemove;
        for (auto it = plans.begin(); ; ++it) 
        {
            auto end = it;
            end++;
            if (end == plans.end()) break;
            if (it->second == end->second) 
            {
                toRemove.insert(end->first);
            }
        }
        for (type_s s : toRemove)
        {
            plans.erase(s);
        }
    }
}
*/