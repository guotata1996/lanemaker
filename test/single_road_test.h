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
        RoadRunner::RoadProfile profile(90 * 100);
        RoadRunner::LaneSection rs1{ RoadRunner::SectionProfile{0, 1}, 0 * 100 };
        RoadRunner::LaneSection rs2{ RoadRunner::SectionProfile{-1, 1}, 30 * 100 };
        profile.AddRightSection(rs1);
        profile.AddRightSection(rs2);

        GenerateAndVerify(profile);
    }

    TEST(SingleRoad, LeftSideOnly)
    {
        RoadRunner::RoadProfile profile(90 * 100);

        RoadRunner::LaneSection ls1{ RoadRunner::SectionProfile{1, 1}, 90 * 100 };
        RoadRunner::LaneSection ls2{ RoadRunner::SectionProfile{0, 1}, 30 * 100 };
        profile.AddLeftSection(ls1);
        profile.AddLeftSection(ls2);

        GenerateAndVerify(profile);
    }

    TEST(SingleRoad, LeftTurnLane)
    {
        RoadRunner::RoadProfile profile(150 * 100);
        profile.AddRightSection({ RoadRunner::SectionProfile{-1, 1}, 0 * 100 });
        profile.AddRightSection({ RoadRunner::SectionProfile{1, 2}, 60 * 100 });
        profile.AddRightSection({ RoadRunner::SectionProfile{-1, 1}, 90 * 100 });

        profile.AddLeftSection({ RoadRunner::SectionProfile{-1, 2}, 150 * 100 });
        profile.AddLeftSection({ RoadRunner::SectionProfile{1, 1}, 120 * 100 });
        profile.AddLeftSection({ RoadRunner::SectionProfile{-1, 2}, 30 * 100 });

        GenerateAndVerify(profile);
    }

    TEST(SingleRoad, LeftTurnLaneCompact)
    {
        RoadRunner::RoadProfile profile(120 * 100);
        profile.AddRightSection({ RoadRunner::SectionProfile{-1, 1}, 0 * 100 });
        profile.AddRightSection({ RoadRunner::SectionProfile{1, 2}, 45 * 100 });
        profile.AddRightSection({ RoadRunner::SectionProfile{-1, 1}, 75 * 100 });

        profile.AddLeftSection({ RoadRunner::SectionProfile{-1, 2}, 120 * 100 });
        profile.AddLeftSection({ RoadRunner::SectionProfile{1, 1}, 90 * 100 });
        profile.AddLeftSection({ RoadRunner::SectionProfile{-1, 2}, 30 * 100 });

        GenerateAndVerify(profile);
    }

    TEST_P(Seed_SingleRoad, SingleRoad) {
        auto seed = GetParam();
        auto road = GenerateConfig(seed);
        GenerateAndVerify(road);
    }
}
