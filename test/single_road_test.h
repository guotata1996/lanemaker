#include <gtest/gtest.h>

#include "road.h"
#include "randomization_utils.h"
#include "validation.h"

#include "Geometries/Line.h"

namespace RoadRunnerTest
{
    void GenerateAndVerify(const RoadRunner::LaneProfile& configs, double refLineLength)
    {
#ifdef G_TEST
        const testing::TestInfo* const test_info =
            testing::UnitTest::GetInstance()->current_test_info();
        std::string outName = test_info->name();
        if (test_info->value_param() != NULL)
        {
            outName = std::string(test_info->value_param());
        }
#else
        std::string outName("output");
#endif
        auto refLine = std::make_unique<odr::Line>(0, 0, 0, 0, refLineLength);
        RoadRunner::Road road(configs, std::move(refLine));
        const odr::Road& gen = road.generated;

#ifndef G_TEST
        odr::OpenDriveMap test_map;
        test_map.id_to_road.insert({ road.ID(), road.generated });
        test_map.export_file("C:\\Users\\guota\\Downloads\\normal_" + outName + ".xodr");
#endif

        Validation::VerifySingleRoad(road.generated);

        road.ReverseRefLine();
        road.ReverseRefLine();
#ifndef G_TEST
        odr::OpenDriveMap test_map2;
        test_map2.id_to_road.insert({ road.ID(), road.generated });
        test_map2.export_file("C:\\Users\\guota\\Downloads\\reverse_" + outName + ".xodr");
#endif
        Validation::VerifySingleRoad(road.generated);
    }

    struct Seed_SingleRoad
        : public testing::TestWithParam<int> {};

    TEST(SingleRoad, RightSideOnly)
    {
        RoadRunner::LaneProfile profile(0, 0, 1, 0);
        profile.OverwriteSection(30.0, 100.0, 100.0, RoadRunner::LanePlan{ 0, 0 }, RoadRunner::LanePlan{-1, 1});

        GenerateAndVerify(profile, 100);
    }

    TEST(SingleRoad, LeftSideOnly)
    {
        RoadRunner::LaneProfile profile(1, 0, 0, 0);
        profile.OverwriteSection(0.0, 30.0, 100.0, RoadRunner::LanePlan{ 1, 1 }, RoadRunner::LanePlan{ 0, 0 });

        GenerateAndVerify(profile, 100);
    }

    TEST_P(Seed_SingleRoad, SingleRoad) {
        auto seed = GetParam();
        const uint32_t Length_M = 100;
        auto road = GenerateConfig(seed, Length_M);
        GenerateAndVerify(road, Length_M);
    }

    INSTANTIATE_TEST_SUITE_P(
        RandomRoad,
        Seed_SingleRoad,
        testing::Range(1, 30));
}
