#pragma once

#ifdef G_TEST
    #define ExpectOrAssert(expr) EXPECT_TRUE(expr)
    #define ExpectNearOrAssert(a, b, epsilon) EXPECT_NEAR(a, b, epsilon)
    #define ExpectLTOrAssert(a, b) EXPECT_LT(a, b)
    #define ExpectGTOrAssert(a, b) EXPECT_GT(a, b)
#else
    #define ExpectOrAssert(expr) if (!(expr)) throw std::logic_error("Validation error: Expect T But F")
    #define ExpectNearOrAssert(a, b, epsilon) if (std::abs(a - b) >= epsilon) throw std::logic_error("Validation error: Expect EQ But NE")
    #define ExpectLTOrAssert(a, b) if (a >= b) throw std::logic_error("Validation error: Expect > But <=")
    #define ExpectGTOrAssert(a, b) if (a <= b) throw std::logic_error("Validation error: Expect < But >=")
#endif

namespace RoadRunnerTest
{
    const double epsilon = 1e-2;
    const double epsilon_integral_result = 1e-2;
}

