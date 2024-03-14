#include <gtest/gtest.h>

#include "junction.h"
#include "junction_verification.h"


namespace RoadRunnerTest
{
    TEST(RoadOperation, ReverseSplitJoin)
    {
        // Create 3 roads, C-(flipped C)
        auto refLine1 = std::make_shared<odr::Line>(0, -30, 0, 0, 60);
        RoadRunner::RoadProfile config(2, 1, 2, -1);
        std::shared_ptr<RoadRunner::Road> r1 = std::make_shared<RoadRunner::Road>(config, refLine1);

        const double R = 30;
        auto refLine2 = std::make_shared<odr::Arc>(0, 50, 20, M_PI_4, 0.75 * M_PI * 2 * R, -1 / R);
        auto rRight = std::make_shared<RoadRunner::Road>(config, refLine2);

        auto refLine3 = std::make_shared<odr::Arc>(0, -50, 20, M_PI_4 * 3, 0.75 * M_PI * 2 * R, 1 / R);
        auto rLeft = std::make_shared<RoadRunner::Road>(config, refLine3);
        // Create right junction, but don't keep its ptr
        auto j2 = std::make_shared<RoadRunner::Junction>();
        j2->CreateFrom({
            RoadRunner::ConnectionInfo{r1, odr::RoadLink::ContactPoint_End},
            RoadRunner::ConnectionInfo{rRight, odr::RoadLink::ContactPoint_Start},
            RoadRunner::ConnectionInfo{rRight, odr::RoadLink::ContactPoint_End}
        });
        std::weak_ptr<RoadRunner::Junction> j2_weak(j2);
        j2.reset();
        // Create left junction, but don't keep its ptr
        auto j1 = std::make_shared<RoadRunner::Junction>();
        j1->CreateFrom({
            RoadRunner::ConnectionInfo{r1, odr::RoadLink::ContactPoint_Start},
            RoadRunner::ConnectionInfo{rLeft, odr::RoadLink::ContactPoint_Start},
            RoadRunner::ConnectionInfo{rLeft, odr::RoadLink::ContactPoint_End}
        });
        std::weak_ptr<RoadRunner::Junction> j1_weak(j1);
        j1.reset();

        // Split r1
        auto part2 = RoadRunner::Road::SplitRoad(r1, 40);
        EXPECT_EQ(r1->generated.predecessor.type, odr::RoadLink::Type_Junction);
        EXPECT_EQ(r1->generated.predecessor.id, j1_weak.lock()->ID());
        EXPECT_EQ(r1->generated.successor.type, odr::RoadLink::Type_None);
        EXPECT_EQ(j1_weak.lock()->formedFrom.size(), 3);

        EXPECT_EQ(part2->generated.predecessor.type, odr::RoadLink::Type_None);
        EXPECT_EQ(part2->generated.successor.type, odr::RoadLink::Type_Junction);
        EXPECT_EQ(part2->generated.successor.id, j2_weak.lock()->ID());
        EXPECT_EQ(j2_weak.lock()->formedFrom.size(), 3);

        VerifyJunction(*j1_weak.lock());
        VerifyJunction(*j2_weak.lock());

        // Join r1
        RoadRunner::Road::JoinRoads(r1, odr::RoadLink::ContactPoint_End, part2, odr::RoadLink::ContactPoint_Start);
        EXPECT_EQ(part2, nullptr);
        EXPECT_EQ(r1->generated.predecessor.type, odr::RoadLink::Type_Junction);
        EXPECT_EQ(r1->generated.predecessor.id, j1_weak.lock()->ID());
        EXPECT_EQ(j1_weak.lock()->formedFrom.size(), 3);

        EXPECT_EQ(r1->generated.successor.type, odr::RoadLink::Type_Junction);
        EXPECT_EQ(r1->generated.successor.id, j2_weak.lock()->ID());
        EXPECT_EQ(j2_weak.lock()->formedFrom.size(), 3);

        VerifyJunction(*j1_weak.lock());
        VerifyJunction(*j2_weak.lock());

        // Reverse r1
        r1->ReverseRefLine();
        EXPECT_EQ(r1->generated.predecessor.type, odr::RoadLink::Type_Junction);
        EXPECT_EQ(r1->generated.predecessor.id, j2_weak.lock()->ID());
        EXPECT_EQ(j2_weak.lock()->formedFrom.size(), 3);

        EXPECT_EQ(r1->generated.successor.type, odr::RoadLink::Type_Junction);
        EXPECT_EQ(r1->generated.successor.id, j1_weak.lock()->ID());
        EXPECT_EQ(j1_weak.lock()->formedFrom.size(), 3);

        VerifyJunction(*j1_weak.lock());
        VerifyJunction(*j2_weak.lock());

        // Reverse rRight
        rRight->ReverseRefLine();
        EXPECT_EQ(j2_weak.lock()->formedFrom.size(), 3);
        VerifyJunction(*j2_weak.lock());

        // Destroy r1
        r1.reset();
        EXPECT_EQ(j1_weak.lock()->formedFrom.size(), 2);
        EXPECT_EQ(j1_weak.lock()->formedFrom.size(), 2);
        VerifyJunction(*j1_weak.lock());
        VerifyJunction(*j2_weak.lock());

        // Destroy remainig
        rRight.reset();
        rLeft.reset();
        EXPECT_EQ(IDGenerator::ForJunction()->size(), 0);
        EXPECT_EQ(IDGenerator::ForRoad()->size(), 0);
    }
}