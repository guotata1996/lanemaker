#include "randomization_utils.h"
#include <math.h>

int RandomIntBetween(int low, int hi)
{
    if (low >= hi) return low;
    int raw = rand();
    return raw % (hi + 1 - low) + low;
}

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

RoadRunner::RoadProfile GenerateConfig(int seed, uint32_t length)
{
    srand(seed);

    const uint32_t MinSection_M = 5;
    RoadRunner::RoadProfile road(1, 0, 1, 0);

    for (int modification = 0; modification != 15; ++modification)
    {
        int mBegin = RandomIntBetween(0, length / MinSection_M) * MinSection_M;
        int mEnd = RandomIntBetween(0, length / MinSection_M) * MinSection_M;
        if (mBegin == mEnd) continue;
        int side = RandomIntBetween(0, 1) == 0 ? -1 : 1;
        int8_t offsetX2 = RandomIntBetween(0, 3) * side;
        uint8_t nLanes = RandomIntBetween(1, 4);
        
        if (side == 1 && mBegin < mEnd ||
            side == -1 && mBegin > mEnd)
        {
            std::swap(mBegin, mEnd);
        }
        road.OverwriteSection(side, mBegin, mEnd, nLanes, offsetX2);
    }

    //std::vector<int> rightS = RandomSortedVector(0, Length_M / MinSection_M, 7);
    //// rightS.pop_back();
    //for (auto sectionBeginIt = rightS.begin();; sectionBeginIt++)
    //{
    //    auto sectionEndIt = sectionBeginIt;
    //    sectionEndIt++;
    //    if (sectionEndIt == rightS.end())
    //    {
    //        break;
    //    }

    //    auto sBegin_cm = 100 * *sectionBeginIt;
    //    auto sEnd_cm = 100 * *sectionEndIt;

    //    road.OverwriteSection(-1, sBegin_cm, sEnd_cm,
    //        RandomIntBetween(1, 4), RandomIntBetween(-3, 0));
    //};

    //std::vector<int> leftS = RandomSortedVector(0, Length_M / MinSection_M, 5);
    //// leftS.pop_back();
    //for (auto sectionBeginIt = leftS.begin();; sectionBeginIt++)
    //{
    //    auto sectionEndIt = sectionBeginIt;
    //    sectionEndIt++;
    //    if (sectionEndIt == leftS.end())
    //    {
    //        break;
    //    }

    //    auto sBegin_cm = 100 * *sectionBeginIt;
    //    auto sEnd_cm = 100 * *sectionEndIt;

    //    RoadRunner::SectionProfile profile;
    //    profile.offsetx2 = RandomIntBetween(0, 3);
    //    profile.laneCount = RandomIntBetween(1, 4);
    //    auto s_cm = 100 * (Length_M - MinSection_M * s);
    //    road.AddLeftSection(RoadRunner::LaneSection{ profile, s_cm });
    //    spdlog::debug("L section at {}: offset={}, lanes={}",
    //        s_cm, profile.offsetx2, profile.laneCount);
    //}

    return road;
}
