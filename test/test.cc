#include <gtest/gtest.h>

#include "lane_profile_test.h"
#include "elevation_profile_test.h"
#include "junction_test.h"
#include "road_geometry_test.h"
#include "road_operation_test.h"
#include "spatial_query_test.h"

namespace LTest
{
    int main(int argc, char** argv) {
        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
    }
}