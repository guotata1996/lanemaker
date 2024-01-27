#include <gtest/gtest.h>

#include "road.h"
#include "verification.h"

void GenerateAndVerify(const RoadRunner::Road& configs)
{
    const testing::TestInfo* const test_info =
        testing::UnitTest::GetInstance()->current_test_info();

    odr::Road gen = (odr::Road)configs;

    odr::OpenDriveMap test_map;
    test_map.id_to_road.insert({ gen.id, gen });
    test_map.export_file("C:\\Users\\guota\\Downloads\\" + std::string(test_info->name()) + ".xodr");

    VerifyLaneWidthinBound(gen);
    VerifySingleRoadLinkage(gen);
    VerifySingleRoadIntegrity(configs, gen);
}

TEST(SingleRoad, MedianWidening) {

    RoadRunner::Road road("");
    road.SetLength(90 * 100);
    RoadRunner::LaneSection rs1{ RoadRunner::RoadProfile{0, 1}, 0 * 100 };
    RoadRunner::LaneSection rs2{ RoadRunner::RoadProfile{-1, 1}, 30 * 100 };
    road.AddRightSection(rs1);
    road.AddRightSection(rs2);

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