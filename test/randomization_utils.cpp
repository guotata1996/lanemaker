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

RoadRunner::LaneProfile GenerateConfig(int seed, uint32_t length)
{
    srand(seed);

    const uint32_t MinSection_M = 5;
    RoadRunner::LaneProfile road(1, 0, 1, 0);

    for (int modification = 0; modification != 15; ++modification)
    {
        int mBegin = RandomIntBetween(0, length / MinSection_M) * MinSection_M;
        int mEnd = RandomIntBetween(0, length / MinSection_M) * MinSection_M;
        if (mBegin == mEnd) continue;
        int8_t offsetX2 = RandomIntBetween(0, 3);
        uint8_t nLanes = RandomIntBetween(1, 4);
        RoadRunner::LanePlan lPlan{ RandomIntBetween(0, 3) , RandomIntBetween(1, 4) };
        RoadRunner::LanePlan rPlan{ -RandomIntBetween(0, 3) , RandomIntBetween(1, 4) };
        
        if (mBegin > mEnd)
        {
            std::swap(mBegin, mEnd);
        }
        road.OverwriteSection(mBegin, mEnd, length, lPlan, rPlan);
    }

    return road;
}
