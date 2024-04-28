#pragma once

#ifdef G_TEST
    #define ExpectOrAssert(expr) EXPECT_TRUE(expr)
    #define ExpectNearOrAssert(a, b, epsilon) EXPECT_NEAR(a, b, epsilon)
    #define ExpectLTOrAssert(a, b) EXPECT_LT(a, b)
    #define ExpectGTOrAssert(a, b) EXPECT_GT(a, b)
#else
    #define ExpectOrAssert(expr) assert(expr)
    #define ExpectNearOrAssert(a, b, epsilon) assert(std::abs(a - b) < epsilon)
    #define ExpectLTOrAssert(a, b) assert(a < b)
    #define ExpectGTOrAssert(a, b) assert(a > b)
#endif

namespace RoadRunnerTest
{
    const double epsilon = 1e-4;
    const double epsilon_integral_result = 1e-2;
}

