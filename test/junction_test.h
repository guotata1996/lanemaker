#include <gtest/gtest.h>

#include "junction.h"
#include "junction_verification.h"
#include <math.h>

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