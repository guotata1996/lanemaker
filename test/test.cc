#include <gtest/gtest.h>

#include "single_road_test.h"
#include "junction_test.h"

namespace RoadRunnerTest
{
    INSTANTIATE_TEST_SUITE_P(
        RandomRoad,
        Seed_SingleRoad,
        testing::Range(1, 30));

    INSTANTIATE_TEST_SUITE_P(
        RandomJunction,
        Seed_SingleJunction,
        testing::Range(1, 100));

    int main(int argc, char** argv) {
        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
    }
}