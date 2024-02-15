#include <gtest/gtest.h>

#include "single_road_test.h"
#include "junction_test.h"

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}