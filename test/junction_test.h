#include <gtest/gtest.h>

#include "junction.h"
#include "junction_verification.h"
#include "randomization_utils.h"
#include <math.h>
#include "MapExporter.h"
#include "Geometries/Arc.h"

namespace RoadRunnerTest
{
    struct Seed_SingleJunction
        : public testing::TestWithParam<int> {};

    TEST_P(Seed_SingleJunction, SingleJunction) {
        auto seed = GetParam();
        srand(seed);

        const int NumRoads = RandomIntBetween(3, 6);
        const double SeparationAngle = M_PI * 2 / NumRoads;
        const type_s RoadLength = 30 * 100;
        const double RoadLengthD = RoadRunner::to_odr_unit(RoadLength);
        const odr::Vec2D nearEnd{ 25, 0 };
        const odr::Vec2D farEnd{ 55, 0 };

        std::vector< RoadRunner::Road> generatedRoads;
        std::vector< bool > incomings;

        int totalOuts = 0;
        std::vector < int> numIncominglanes;
        for (int i = 0; i != NumRoads; ++i)
        {
            int8_t rightLanes = RandomIntBetween(1, 4);
            int8_t leftLanes = RandomIntBetween(1, 4);
            int8_t rightOffset = RandomIntBetween(-1, 0);
            int8_t leftOffset = RandomIntBetween(0, 1);
            bool incoming = RandomIntBetween(0, 1) == 0;  //refLane pointing junction
            totalOuts += incoming ? leftLanes : rightLanes;
            numIncominglanes.push_back(incoming ? rightLanes : leftLanes);

            RoadRunner::RoadProfile cfg(leftLanes, leftOffset, rightLanes, rightOffset);

            odr::Vec2D refLineOrigin = incoming ? farEnd : nearEnd;
            refLineOrigin = odr::rotateCCW(refLineOrigin, SeparationAngle * i);
            double hdg = SeparationAngle * i;
            if (incoming) hdg += M_PI;
            auto refLine = std::make_shared<odr::Line>(0, refLineOrigin[0], refLineOrigin[1], hdg, RoadLengthD);

            RoadRunner::Road road(cfg, refLine);
            generatedRoads.push_back(std::move(road));
            incomings.push_back(incoming);
        }

        std::vector< RoadRunner::ConnectionInfo> connectionInfo;
        for (int i = 0; i != NumRoads; ++i)
        {
            connectionInfo.push_back(RoadRunner::ConnectionInfo{
                &generatedRoads[i], incomings[i] ? RoadLengthD : 0 });
        }

        RoadRunner::Junction j1(connectionInfo);
        // RoadRunner::MapExporter odr_writer("C:\\Users\\guota\\Downloads\\junction_" + std::to_string(seed) + ".xodr");
        // odr_writer.Update();

        int mostIncomingLanes = *std::max_element(numIncominglanes.begin(), numIncominglanes.end());
        if (mostIncomingLanes > totalOuts)
        {
            EXPECT_EQ(j1.generationError, RoadRunner::JunctionError::Junction_TooManyIncomingLanes);
        }
        else
        {
            VerifyJunction(j1, connectionInfo);
        }
    }

    TEST(SingleJunction, BothEnds)
    {
        RoadRunner::RoadProfile commonProfile(2, 0, 2, 0);
        auto circleRRef = std::make_shared<odr::Arc>(0, 20, 0, 0, 2 * M_PI * 20 * 0.75, 1 / 20.0);
        RoadRunner::Road circleR(commonProfile, circleRRef);
        auto circleLRef = std::make_shared<odr::Arc>(0, 0, -20, M_PI_2 * 3, 2 * M_PI * 20 * 0.75, -1 / 20.0);
        RoadRunner::Road circleL(commonProfile, circleLRef);
       
        std::vector< RoadRunner::ConnectionInfo> connectionInfo
        {
            RoadRunner::ConnectionInfo{&circleL, 0},
            RoadRunner::ConnectionInfo{&circleL, circleL.Length()},
            RoadRunner::ConnectionInfo{&circleR, 0},
            RoadRunner::ConnectionInfo{&circleR, circleR.Length()}
        };

        RoadRunner::Junction j1(connectionInfo);
        //RoadRunner::MapExporter odr_writer("C:\\Users\\guota\\Downloads\\junction_both.xodr");
        //odr_writer.Update();

        VerifyJunction(j1, connectionInfo);
    }

    INSTANTIATE_TEST_SUITE_P(
        RandomJunction,
        Seed_SingleJunction,
        testing::Range(1, 100));
}
