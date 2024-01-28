#include <gtest/gtest.h>

#include "road.h"
#include "verification.h"
#include "test_randomization.h"

TEST(SingleRoad, RightSideOnly) {

    RoadRunner::Road road("");
    road.SetLength(90 * 100);
    RoadRunner::LaneSection rs1{ RoadRunner::RoadProfile{0, 1}, 0 * 100 };
    RoadRunner::LaneSection rs2{ RoadRunner::RoadProfile{-1, 1}, 30 * 100 };
    road.AddRightSection(rs1);
    road.AddRightSection(rs2);

    GenerateAndVerify(road);
}

TEST(SingleRoad, LeftSideOnly)
{
    RoadRunner::Road road("");
    road.SetLength(90 * 100);

    RoadRunner::LaneSection ls1{ RoadRunner::RoadProfile{1, 1}, 90 * 100 };
    RoadRunner::LaneSection ls2{ RoadRunner::RoadProfile{0, 1}, 30 * 100 };
    road.AddLeftSection(ls1);
    road.AddLeftSection(ls2);

    GenerateAndVerify(road);
}

TEST(SingleRoad, LeftTurnLane)
{
    RoadRunner::Road road("");
    road.SetLength(150 * 100);
    road.AddRightSection({ RoadRunner::RoadProfile{-1, 1}, 0 * 100 });
    road.AddRightSection({ RoadRunner::RoadProfile{1, 2}, 60 * 100 });
    road.AddRightSection({ RoadRunner::RoadProfile{-1, 1}, 90 * 100 });

    road.AddLeftSection({ RoadRunner::RoadProfile{-1, 2}, 150 * 100 });
    road.AddLeftSection({ RoadRunner::RoadProfile{1, 1}, 120 * 100 });
    road.AddLeftSection({ RoadRunner::RoadProfile{-1, 2}, 30 * 100 });

    GenerateAndVerify(road);
}

TEST(SingleRoad, LeftTurnLaneCompact)
{
    RoadRunner::Road road("");
    road.SetLength(120 * 100);
    road.AddRightSection({ RoadRunner::RoadProfile{-1, 1}, 0 * 100 });
    road.AddRightSection({ RoadRunner::RoadProfile{1, 2}, 45 * 100 });
    road.AddRightSection({ RoadRunner::RoadProfile{-1, 1}, 75 * 100 });


    road.AddLeftSection({ RoadRunner::RoadProfile{-1, 2}, 120 * 100 });
    road.AddLeftSection({ RoadRunner::RoadProfile{1, 1}, 90 * 100 });
    road.AddLeftSection({ RoadRunner::RoadProfile{-1, 2}, 30 * 100 });

    GenerateAndVerify(road);
}


struct TestSeed
    : public testing::TestWithParam<int> {};

TEST_P(TestSeed, SingleRoad) {
    auto seed = GetParam();
    auto road = GenerateConfig(seed);
    GenerateAndVerify(road);
}

std::vector<int> seeds(3);

INSTANTIATE_TEST_SUITE_P(
    RandomRoadGen,
    TestSeed,
    testing::Range(10, 30));