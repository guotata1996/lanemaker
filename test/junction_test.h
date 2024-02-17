#include <gtest/gtest.h>

#include "junction.h"
#include "junction_verification.h"
#include "randomization_utils.h"
#include <math.h>
#include "MapExporter.h"

namespace RoadRunnerTest
{
    struct Seed_SingleJunction
        : public testing::TestWithParam<int> {};

    TEST(Junction, X1) {
        RoadRunner::RoadProfile vConfig(40 * 100);
        vConfig.AddRightSection({ RoadRunner::SectionProfile{-1, 3} , 0 * 100 });
        vConfig.AddLeftSection({ RoadRunner::SectionProfile{1, 2} , 40 * 100 });
        RoadRunner::RoadProfile hConfig(40 * 100);
        hConfig.AddRightSection({ RoadRunner::SectionProfile{-1, 1}, 0 * 100 });
        hConfig.AddLeftSection({ RoadRunner::SectionProfile{1, 2}, 40 * 100 });

        RoadRunner::Road upper(vConfig);
        RoadRunner::Road bottom(vConfig);

        RoadRunner::Road left(hConfig);
        RoadRunner::Road right(hConfig);

        bottom.Generate(odr::Line(0, 0, -60, M_PI / 2, bottom.Length()));
        upper.Generate(odr::Line(0, 0, 20, M_PI / 2, upper.Length()));
        left.Generate(odr::Line(0, -60, 0, 0, left.Length()));
        right.Generate(odr::Line(0, 20, 0, 0, right.Length()));

        std::vector<RoadRunner::ConnectionInfo> connectionInfo({
            RoadRunner::ConnectionInfo{&upper, 0},
            RoadRunner::ConnectionInfo{&bottom, bottom.Length()},
            RoadRunner::ConnectionInfo{&left, left.Length()},
            RoadRunner::ConnectionInfo{&right, 0} });

        RoadRunner::Junction j1(connectionInfo);
        VerifyJunction(j1, connectionInfo);
    }

    TEST_P(Seed_SingleJunction, SingleJunction) {
        auto seed = GetParam();
        srand(seed);

        const int NumRoads = RandomIntBetween(3, 6);
        const double SeparationAngle = M_PI * 2 / NumRoads;
        const uint32_t RoadLength = 30 * 100;
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

            RoadRunner::RoadProfile cfg;
            if (rightLanes != 0)
                cfg.AddRightSection({ RoadRunner::SectionProfile{rightOffset, rightLanes} , 0 });
            if (leftLanes != 0)
                cfg.AddLeftSection({ RoadRunner::SectionProfile{leftOffset, leftLanes} , RoadLength });

            odr::Vec2D refLineOrigin = incoming ? farEnd : nearEnd;
            refLineOrigin = odr::rotateCCW(refLineOrigin, SeparationAngle * i);
            double hdg = SeparationAngle * i;
            if (incoming) hdg += M_PI;
            auto refLine = odr::Line(0, refLineOrigin[0], refLineOrigin[1], hdg, RoadLengthD);

            RoadRunner::Road road(cfg);
            road.Generate(refLine);
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
}
