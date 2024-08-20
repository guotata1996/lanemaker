#include <gtest/gtest.h>

#include "randomization_utils.h"
#include "Geometries/CubicSpline.h"
#include "validation.h"
#include "test_const.h"

using SplineGen = RoadRunner::CubicSplineGenerator;

namespace RoadRunnerTest
{
	// Not clear why this needs to be re-defined
	void Validation::VerifySingleRoadElevation(const odr::CubicSpline& eProfile)
	{
		for (auto it = eProfile.s0_to_poly.begin(); it != eProfile.s0_to_poly.end(); ++it)
		{
			auto next = it;
			next++;
			if (next == eProfile.s0_to_poly.end()) break;
			auto s = next->first;
			ExpectNearOrAssert(it->second.get(s), next->second.get(s), epsilon);
		}
	}

	TEST(ElevationProfile, SampleUsage)
	{
		odr::CubicSpline ePeofile;
		SplineGen::OverwriteSection(ePeofile, 100.0, 10.0, 40.0, 4.1);
		SplineGen::OverwriteSection(ePeofile, 100.0, 20.0, 40.0, 7.9);
		Validation::VerifySingleRoadElevation(ePeofile);
		EXPECT_NEAR(ePeofile.get(0), 0, epsilon);
		EXPECT_NEAR(ePeofile.get(10), 4.1, epsilon);
		EXPECT_NEAR(ePeofile.get(20), 7.9, epsilon);
		EXPECT_NEAR(ePeofile.get(40), 7.9, epsilon);
		EXPECT_NEAR(ePeofile.get(100), 0, epsilon);
	}

	struct RandElevationParam
		: public testing::TestWithParam<int> {};

	TEST_P(RandElevationParam, ElevationProfile) {
		auto seed = GetParam();
		srand(seed);

		const uint32_t Length = 100;
		const uint32_t UnitLen = 5;
		odr::CubicSpline eProfile;
		for (int i = 0; i != 15; ++i)
		{
			int mul1 = RandomIntBetween(0, 20);
			int mul2 = RandomIntBetween(0, 20);
			if (mul1 == mul2) continue;
			if (mul1 > mul2) std::swap(mul1, mul2);
			SplineGen::OverwriteSection(eProfile, Length, mul1 * UnitLen, mul2 * UnitLen, RandomIntBetween(-10, 10));
		}
		Validation::VerifySingleRoadElevation(eProfile);

		double probeValues[Length + 1]; // values taken from original spline at s = 0,1,...,100 
		for (int i = 0; i <= Length; ++i)
		{
			probeValues[i] = eProfile.get(i);
		}

		// Test reverse
		eProfile.reverse(Length);
		for (int i = 0; i <= Length; ++i)
		{
			auto valFromRev = eProfile.get(Length - i);
			ExpectNearOrAssert(probeValues[i], valFromRev, epsilon);
		}
		eProfile.reverse(Length);

		// Test split
		int splitPoint = RandomIntBetween(1, Length - 1);
		odr::CubicSpline second = eProfile.split(splitPoint);
		for (int i = 0; i <= splitPoint; ++i)
		{
			auto valFromFirst = eProfile.get(i);
			ExpectNearOrAssert(probeValues[i], valFromFirst, epsilon);
		}
		for (int i = splitPoint; i <= Length; ++i)
		{
			auto valFromSecond = second.get(i - splitPoint);
			ExpectNearOrAssert(probeValues[i], valFromSecond, epsilon);
		}

		// Test join
		eProfile.join(splitPoint, second);
		for (int i = 0; i <= Length; ++i)
		{
			auto valFromJoin = eProfile.get(i);
			ExpectNearOrAssert(probeValues[i], valFromJoin, epsilon);
		}
	}

	INSTANTIATE_TEST_SUITE_P(
		RandomElevation,
		RandElevationParam,
		testing::Range(1, 30));
}