#include <gtest/gtest.h>

#include "Geometries/Line.h"
#include "Geometries/Arc.h"
#include "Geometries/Spiral.h"
#include "Geometries/ParamPoly3.h"

#include "spdlog/spdlog.h"

namespace RoadRunnerTest
{
    constexpr int NGeometry = 5;
    const std::unique_ptr< RoadGeometry> TestingGeo[NGeometry]
    {
        std::make_unique<odr::Line>(0, 10, 20, 2.34, 40),
        std::make_unique<odr::Arc>(0, 20, 30, 1.45, 40, 1.0 / 30),
        std::make_unique<odr::Spiral>(0, 20, 10, 2.55, 50, 1.0 / 25, 1.0 / 50),
        std::make_unique<odr::Spiral>(0, 20, 10, 0.34, 50, -1.0 / 25, -1.0 / 50),
        std::make_unique<odr::ParamPoly3>(0, -5, -15, 1.57, 47.016827930298923,
                0, 60, -60, 40, 0, 0, 0, -20) // [a-d][U/V]
    };

    constexpr int Subdivision = 20;

    TEST(RoadGeometry, Reverse)
    {
        for (int geoIndex = 0; geoIndex != NGeometry; ++geoIndex)
        {
            auto geo = TestingGeo[geoIndex]->clone();
            auto rev = geo->clone();
            rev->reverse();

            for (int i = 0; i <= Subdivision; ++i)
            {
                double probeS = static_cast<double>(i) / Subdivision * geo->length;
                double probeSRev = geo->length - probeS;

                auto p1 = geo->get_xy(probeS);
                auto g1 = geo->get_grad(probeS);
                auto p2 = rev->get_xy(probeSRev);
                auto g2 = rev->get_grad(probeSRev);
                double pd = odr::euclDistance(p1, p2);
                double gd = odr::euclDistance(g1, odr::negate(g2));
                EXPECT_LT(pd, epsilon_integral_result);
                EXPECT_LT(gd, epsilon_integral_result);
            }
        }
    }

    TEST(RoadGeometry, Rebase)
    {
        for (int geoIndex = 0; geoIndex != NGeometry; ++geoIndex)
        {
            for (int i = 1; i < Subdivision; ++i)
            {
                auto& geo = TestingGeo[geoIndex];
                auto clone = geo->clone();
                double splitS = static_cast<double>(i) / Subdivision * geo->length;
                clone->rebase(splitS);

                for (int j = i; j <= Subdivision; ++j)
                {
                    double checkSOnGeo = static_cast<double>(j) / Subdivision * geo->length;
                    double checkSOnClone = checkSOnGeo - splitS;
                    auto p1 = geo->get_xy(checkSOnGeo);
                    auto g1 = geo->get_grad(checkSOnGeo);
                    auto h1 = std::atan2(g1[1], g1[0]);
                    auto p2 = clone->get_xy(checkSOnClone);
                    auto g2 = clone->get_grad(checkSOnClone);
                    auto h2 = std::atan2(g2[1], g2[0]);
                    double pd = odr::euclDistance(p1, p2);
                    EXPECT_LT(pd, epsilon_integral_result);
                    EXPECT_NEAR(h1, h2, epsilon_integral_result);
                }
            }
        }
    }

    TEST(RoadGeometry, Trim)
    {
        for (int geoIndex = 0; geoIndex != NGeometry; ++geoIndex)
        {
            for (int i = 1; i < Subdivision; ++i)
            {
                auto& geo = TestingGeo[geoIndex];
                auto clone = geo->clone();
                double splitS = static_cast<double>(i) / Subdivision * geo->length;
                clone->trim(splitS);

                for (int j = 0; j <= i; ++j)
                {
                    double checkSOnGeo = static_cast<double>(j) / Subdivision * geo->length;
                    double checkSOnClone = checkSOnGeo;
                    auto p1 = geo->get_xy(checkSOnGeo);
                    auto g1 = geo->get_grad(checkSOnGeo);
                    auto h1 = std::atan2(g1[1], g1[0]);
                    auto p2 = clone->get_xy(checkSOnClone);
                    auto g2 = clone->get_grad(checkSOnClone);
                    auto h2 = std::atan2(g2[1], g2[0]);
                    double pd = odr::euclDistance(p1, p2);
                    EXPECT_LT(pd, epsilon_integral_result);
                    EXPECT_NEAR(h1, h2, epsilon_integral_result);
                }
            }
        }
    }

    std::shared_ptr<RoadRunner::Road> GenerateComplexRoad()
    {
        auto refLine1 = std::make_shared<odr::Line>(0, 0, 0, M_PI_2, 40);
        RoadRunner::RoadProfile config(2, 0, 2, 0);
        auto r1 = std::make_shared<RoadRunner::Road>(config, refLine1); // (0, 40)

        {
            auto refLine2 = std::make_shared<odr::Arc>(0, 0, 50, M_PI_2, 60, -1 / 20.0); // about (40, 50)
            auto r2 = std::make_shared<RoadRunner::Road>(config, refLine2);
            int err = RoadRunner::Road::JoinRoads(r1, odr::RoadLink::ContactPoint_End, r2, odr::RoadLink::ContactPoint_Start);
        }

        {
            auto refLine3 = std::make_shared<odr::Spiral>(0, 90, 10, M_PI / 8, 50, 1 / 60.0, 1 / 30.0);
            auto r3 = std::make_shared<RoadRunner::Road>(config, refLine3);
            int err = RoadRunner::Road::JoinRoads(r1, odr::RoadLink::ContactPoint_End, r3, odr::RoadLink::ContactPoint_Start);
        }

        {
            auto refLine4 = std::make_shared<odr::Line>(0, 50, 80, 0, 30);
            auto r4 = std::make_shared<RoadRunner::Road>(config, refLine4);
            int err = RoadRunner::Road::JoinRoads(r1, odr::RoadLink::ContactPoint_End, r4, odr::RoadLink::ContactPoint_End);
        }

        return r1;
    }

    void VerifyReverseMatch(std::shared_ptr<RoadRunner::Road> &r1)
    {
        std::map<double, odr::Vec2D> originalSToPos;
        std::map<double, double> originalSToHdg;
        for (int i = 0; i <= Subdivision; ++i)
        {
            double s = static_cast<double>(i) / Subdivision * r1->Length();
            originalSToPos.emplace(s, r1->generated.ref_line.get_xy(s));
            auto grad = r1->generated.ref_line.get_grad_xy(s);
            originalSToHdg.emplace(s, std::atan2(grad[1], grad[0]));
        }

        r1->ReverseRefLine();
        for (const auto& sAndPos : originalSToPos)
        {
            double s = sAndPos.first;
            odr::Vec2D expectedPos = sAndPos.second;
            double expectedHdg = originalSToHdg.at(s) + M_PI;
            
            if (expectedHdg >= M_PI) expectedHdg -= 2 * M_PI;
            double sOnReverse = r1->Length() - s;
            odr::Vec2D realPos = r1->generated.ref_line.get_xy(sOnReverse);
            auto realGrad = r1->generated.ref_line.get_grad_xy(sOnReverse);
            double realHdg = std::atan2(realGrad[1], realGrad[0]);
            double pd = odr::euclDistance(expectedPos, realPos);
            EXPECT_LT(pd, epsilon_integral_result) << " Eval error at " << s;
            EXPECT_NEAR(expectedHdg, realHdg, epsilon_integral_result) << " Eval error at " << s;
        }
    }

    struct Split_Reverse_RoadGeometry
        : public testing::TestWithParam<int> {};
    TEST_P(Split_Reverse_RoadGeometry, RoadGeometry)
    {
        auto r1 = GenerateComplexRoad();
        std::map<double, odr::Vec2D> originalSToPos;
        std::map<double, double> originalSToHdg;
        for (int i = 0; i <= Subdivision; ++i)
        {
            double s = static_cast<double>(i) / Subdivision * r1->Length();
            originalSToPos.emplace(s, r1->generated.ref_line.get_xy(s));
            auto grad = r1->generated.ref_line.get_grad_xy(s);
            originalSToHdg.emplace(s, std::atan2(grad[1], grad[0]));
        }

        auto splitIndex = GetParam();
        double splitS = static_cast<double>(splitIndex) / Subdivision * r1->Length();

        std::shared_ptr<RoadRunner::Road> r2 = std::move(RoadRunner::Road::SplitRoad(r1, splitS));
        for (const auto& sAndPos : originalSToPos)
        {
            double s = sAndPos.first;
            odr::Vec2D expectedPos = sAndPos.second;
            double expectedHdg = originalSToHdg.at(s);

            odr::Vec2D realPos, realGrad;
            int rIndex;
            if (s < r1->Length())
            {
                realPos = r1->generated.ref_line.get_xy(s);
                realGrad = r1->generated.ref_line.get_grad_xy(s);
                rIndex = 1;
            }
            else
            {
                realPos = r2->generated.ref_line.get_xy(s - r1->Length());
                realGrad = r2->generated.ref_line.get_grad_xy(s - r1->Length());
                rIndex = 2;
            }
            double realHdg = std::atan2(realGrad[1], realGrad[0]);

            double pd = odr::euclDistance(expectedPos, realPos);
            EXPECT_LT(pd, epsilon_integral_result) << " Eval error at " << s << " From r" << rIndex << " :" << r1->Length();
            EXPECT_NEAR(expectedHdg, realHdg, epsilon_integral_result) << " Eval error at " << s;
        }

        VerifyReverseMatch(r1);
        VerifyReverseMatch(r2);

        VerifyReverseMatch(r1);
        VerifyReverseMatch(r2);
    }

    INSTANTIATE_TEST_SUITE_P(
        CutRefLine,
        Split_Reverse_RoadGeometry,
        testing::Range(1, Subdivision - 1));

    // TODO: Verify profile
}