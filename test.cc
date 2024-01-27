#include <gtest/gtest.h>

#include "road.h"

// Demonstrate some basic assertions.
TEST(SingleRoad, MedianWidening) {
    // Expect two strings not to be equal.
    //EXPECT_STRNE("hello", "world");
    //// Expect equality.
    //EXPECT_EQ(7 * 6, 42);

    odr::OpenDriveMap test_map;

    RoadRunner::Road road("1");
    road.SetLength(90 * 100);
    RoadRunner::LaneSection rs1{ RoadRunner::RoadProfile{0, 1}, 0 * 100 };
    RoadRunner::LaneSection rs2{ RoadRunner::RoadProfile{-1, 1}, 30 * 100 };
    road.AddRightSection(rs1);
    road.AddRightSection(rs2);

    RoadRunner::LaneSection ls1{ RoadRunner::RoadProfile{1, 1}, 90 * 100 };
    RoadRunner::LaneSection ls2{ RoadRunner::RoadProfile{0, 1}, 30 * 100 };
    road.AddLeftSection(ls1);
    road.AddLeftSection(ls2);

    odr::Road exportRoad = (odr::Road)road;
    test_map.id_to_road.insert({ exportRoad.id, exportRoad });
    test_map.export_file("C:\\Users\\guota\\Downloads\\test_median_widen.xodr");
}

TEST(SingleRoad, LeftTurnLane)
{
    odr::OpenDriveMap test_map;

    RoadRunner::Road road("1");
    road.SetLength(150 * 100);
    road.AddRightSection({ RoadRunner::RoadProfile{-1, 1}, 0 * 100 });
    road.AddRightSection({ RoadRunner::RoadProfile{1, 2}, 60 * 100 });
    road.AddRightSection({ RoadRunner::RoadProfile{-1, 1}, 90 * 100 });

    road.AddLeftSection({ RoadRunner::RoadProfile{-1, 2}, 150 * 100 });
    road.AddLeftSection({ RoadRunner::RoadProfile{1, 1}, 120 * 100 });
    road.AddLeftSection({ RoadRunner::RoadProfile{-1, 2}, 30 * 100 });

    odr::Road exportRoad = (odr::Road)road;
    test_map.id_to_road.insert({ exportRoad.id, exportRoad });
    test_map.export_file("C:\\Users\\guota\\Downloads\\test_left_turn_lane_1.xodr");
}

TEST(SingleRoad, LeftTurnLaneCompact)
{
    odr::OpenDriveMap test_map;

    RoadRunner::Road road("1");
    road.SetLength(120 * 100);
    road.AddRightSection({ RoadRunner::RoadProfile{-1, 1}, 0 * 100 });
    road.AddRightSection({ RoadRunner::RoadProfile{1, 2}, 45 * 100 });
    road.AddRightSection({ RoadRunner::RoadProfile{-1, 1}, 75 * 100 });


    road.AddLeftSection({ RoadRunner::RoadProfile{-1, 2}, 120 * 100 });
    road.AddLeftSection({ RoadRunner::RoadProfile{1, 1}, 90 * 100 });
    road.AddLeftSection({ RoadRunner::RoadProfile{-1, 2}, 30 * 100 });


    odr::Road exportRoad = (odr::Road)road;
    test_map.id_to_road.insert({ exportRoad.id, exportRoad });
    test_map.export_file("C:\\Users\\guota\\Downloads\\test_left_turn_lane_2.xodr");
}