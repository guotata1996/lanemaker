#pragma once

#include "road.h"

#include "spdlog/spdlog.h"

#ifdef G_TEST
#include <gtest/gtest.h>
#endif

// Inclusive
int RandomIntBetween(int low, int hi)
{
    if (low >= hi) return low;
    int raw = rand();
    return raw % (hi + 1 - low) + low;
}

// Sorted, Containing low and hi
std::vector<int> RandomSortedVector(int low, int hi, uint32_t count)
{
    assert(count >= 2);

    std::set<int> _set;
    _set.insert(low);
    _set.insert(hi);
    while (_set.size() < count)
    {
        _set.insert(RandomIntBetween(low, hi));
    }
    return std::vector<int>(_set.begin(), _set.end());
}

RoadRunner::Road GenerateConfig(int seed)
{
    srand(seed);
    RoadRunner::Road road(std::to_string(seed));

    const uint32_t Length_M = 100;
    const uint32_t MinSection_M = 5;

    road.SetLength(Length_M * 100);

    std::vector<int> rightS = RandomSortedVector(0, Length_M / MinSection_M, 7);
    rightS.pop_back();
    for (uint32_t s : rightS)
    {
        RoadRunner::RoadProfile profile;
        profile.offsetx2 = RandomIntBetween(-3, 0);
        profile.laneCount = RandomIntBetween(1, 4);
        auto s_cm = 100 * MinSection_M * s;
        road.AddRightSection(RoadRunner::LaneSection{ profile, s_cm });
        spdlog::debug("R section at {}: offset={}, lanes={}",
            s_cm, profile.offsetx2, profile.laneCount);
    }

    std::vector<int> leftS = RandomSortedVector(0, 100 / MinSection_M, 5);
    leftS.pop_back();
    for (uint32_t s : leftS)
    {
        RoadRunner::RoadProfile profile;
        profile.offsetx2 = RandomIntBetween(0, 3);
        profile.laneCount = RandomIntBetween(1, 4);
        auto s_cm = 100 * (Length_M - MinSection_M * s);
        road.AddLeftSection(RoadRunner::LaneSection{ profile, s_cm });
        spdlog::debug("L section at {}: offset={}, lanes={}",
            s_cm, profile.offsetx2, profile.laneCount);
    }

    return road;
}