#include <gtest/gtest.h>

#include "road.h"

// Demonstrate some basic assertions.
TEST(SingleRoad, MedianWidening) {
    // Expect two strings not to be equal.
    //EXPECT_STRNE("hello", "world");
    //// Expect equality.
    //EXPECT_EQ(7 * 6, 42);

    odr::OpenDriveMap test_map;

    RoadRunnder::Road road("1");
    road.SetLength(90 * 100);
    RoadRunnder::LaneSection rs1{ RoadRunnder::RoadProfile{0, 1}, 0 * 100 };
    RoadRunnder::LaneSection rs2{ RoadRunnder::RoadProfile{-1, 1}, 30 * 100 };
    road.AddRightSection(rs1);
    road.AddRightSection(rs2);

    RoadRunnder::LaneSection ls1{ RoadRunnder::RoadProfile{1, 1}, 0 * 100 };
    RoadRunnder::LaneSection ls2{ RoadRunnder::RoadProfile{0, 1}, 60 * 100 };
    road.AddLeftSection(ls1);
    road.AddLeftSection(ls2);

    odr::Road exportRoad = (odr::Road)road;
    test_map.id_to_road.insert({ exportRoad.id, exportRoad });
    test_map.export_file("C:\\Users\\guota\\Downloads\\test_median_widen.xodr");
}

TEST(SingleRoad, LeftTurnLane)
{
    odr::OpenDriveMap test_map;

    RoadRunnder::Road road("1");
    road.SetLength(150 * 100);
    road.AddRightSection({ RoadRunnder::RoadProfile{-1, 1}, 0 * 100 });
    road.AddRightSection({ RoadRunnder::RoadProfile{1, 2}, 60 * 100 });
    road.AddRightSection({ RoadRunnder::RoadProfile{-1, 1}, 90 * 100 });

    road.AddLeftSection({ RoadRunnder::RoadProfile{-1, 2}, 0 * 100 });
    road.AddLeftSection({ RoadRunnder::RoadProfile{1, 1}, 30 * 100 });
    road.AddLeftSection({ RoadRunnder::RoadProfile{-1, 2}, 120 * 100 });

    odr::Road exportRoad = (odr::Road)road;
    test_map.id_to_road.insert({ exportRoad.id, exportRoad });
    test_map.export_file("C:\\Users\\guota\\Downloads\\test_left_turn_lane_1.xodr");
}