#include <gtest/gtest.h>

#include "road.h"
#include "randomization_utils.h"
#include "road_verification.h"

namespace RoadRunnerTest
{
    struct Seed_SingleRoad
        : public testing::TestWithParam<int> {};

    TEST(SingleRoad, RightSideOnly)
    {
        RoadRunner::RoadProfile profile(0, 0, 1, 0);
        profile.OverwriteSection(-1, 30, 100, 1, -1);

        GenerateAndVerify(profile, 100);
    }

    TEST(SingleRoad, LeftSideOnly)
    {
        RoadRunner::RoadProfile profile(1, 0, 0, 0);
        profile.OverwriteSection(1, 30, 0, 1, 1);

        GenerateAndVerify(profile, 100);
    }

    TEST_P(Seed_SingleRoad, SingleRoad) {
        auto seed = GetParam();
        const uint32_t Length_M = 100;
        auto road = GenerateConfig(seed, Length_M);
        GenerateAndVerify(road, Length_M);
    }
}
