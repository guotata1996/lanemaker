#include "curve_fitting.h"

#include <map>
#include <chrono>
#include <vector>

#include <CGAL/intersections.h>
#include <CGAL/Cartesian.h>

#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
#include "Geometries/Arc.h"
#include "Geometries/Spiral.h"
#include "CubicBezier.hpp"
#include "spdlog/spdlog.h"


#ifdef G_TEST
#include <gtest/gtest.h>
#endif


using namespace std::chrono;

using namespace CGAL;

typedef Cartesian<double>  Kernel;

namespace LM
{
    std::unique_ptr<odr::RoadGeometry> ConnectRays(const odr::Vec2D& startPos, const odr::Vec2D& startHdg,
        const odr::Vec2D& endPos, const odr::Vec2D& endHdg)
    {
        auto spiral = FitSpiral(startPos, startHdg, endPos, endHdg);
        if (spiral != nullptr)
        {
            return spiral;
        }
        return FitParamPoly(startPos, startHdg, endPos, endHdg);
    }

    // l1 --> conn --> l2
    std::unique_ptr<odr::RoadGeometry> FitParamPoly(const odr::Vec2D& startPos, const odr::Vec2D& startHdg,
        const odr::Vec2D& endPos, const odr::Vec2D& endHdg)
    {
        double hdg = std::atan2(startHdg[1], startHdg[0]);
        Point_2 <Kernel> p1(startPos[0], startPos[1]), p2(endPos[0], endPos[1]);
        Direction_2< Kernel > dir1(startHdg[0], startHdg[1]);
        Direction_2< Kernel> dir2(endHdg[0], endHdg[1]);
        Ray_2< Kernel > ray1(p1, dir1);
        Ray_2< Kernel > ray2(p2, -dir2);
        bool p2OnRay1 = squared_distance(ray1, p2) < std::pow(1e-4, 2);
        bool p1OnRay2 = squared_distance(ray2, p1) < std::pow(1e-4, 2);

        if (p2OnRay1 && p1OnRay2)
        {
            // collinear case
            return std::make_unique<odr::Line>(0, startPos[0], startPos[1], hdg, odr::euclDistance(startPos, endPos));
        }

        if (!p2OnRay1 && !p1OnRay2)
        {
            Object result = intersection(ray1, ray2);

            if (const CGAL::Point_2<Kernel>* pm = object_cast<Point_2<Kernel>>(&result))
            {
                // Converging case - 2nd order
                odr::Vec2D odr_p0{ 0, 0 };
                odr::Vec2D odr_p1{ pm->x(), pm->y() };
                odr_p1 = odr::toLocal(odr_p1, startPos, startHdg);
                odr::Vec2D odr_p2({ p2.x(), p2.y() });
                odr_p2 = odr::toLocal(odr_p2, startPos, startHdg);

                if (odr::euclDistance(odr_p0, odr_p1) < odr::euclDistance(odr_p0, odr_p2) * 5)
                {
                    // project against very far intersection point, i.e. nearly parallel hdg
                    odr::CubicBezier2D curve({ odr_p0, odr_p1, odr_p1, odr_p2 });
                    auto coefficients = odr::CubicBezier2D::get_coefficients(curve.control_points);
                    return std::make_unique<odr::ParamPoly3>(0, startPos[0], startPos[1], hdg, curve.get_length(),
                        coefficients[0][0], coefficients[1][0], coefficients[2][0], coefficients[3][0],
                        coefficients[0][1], coefficients[1][1], coefficients[2][1], coefficients[3][1],
                        true);
                }
            }
        }

        // diverging case - 3rd order
        double dist = std::sqrt(squared_distance(p1, p2));
        Point_2 <Kernel> c1 = p1;
        c1 += dir1.to_vector() * dist / 2;
        odr::Vec2D odr_p0{ 0, 0 };
        odr::Vec2D odr_p1({ c1.x(), c1.y() });
        odr_p1 = odr::toLocal(odr_p1, startPos, startHdg);
        Point_2 <Kernel> c2 = p2;
        c2 -= dir2.to_vector() * dist / 2;
        odr::Vec2D odr_p2{ c2.x(), c2.y() };
        odr_p2 = odr::toLocal(odr_p2, startPos, startHdg);
        odr::Vec2D odr_p3{ p2.x(), p2.y() };
        odr_p3 = odr::toLocal(odr_p3, startPos, startHdg);
        odr::CubicBezier2D curve({ odr_p0, odr_p1, odr_p2, odr_p3 });

        auto coefficients = odr::CubicBezier2D::get_coefficients(curve.control_points);
        return std::make_unique<odr::ParamPoly3>(0, startPos[0], startPos[1], hdg, curve.get_length(),
            coefficients[0][0], coefficients[1][0], coefficients[2][0], coefficients[3][0],
            coefficients[0][1], coefficients[1][1], coefficients[2][1], coefficients[3][1],
            true);
    }

    template<typename F, typename T>
    bool FindZeroCrossing(F function, T& x0, double tol = 7e-5)
    {
        // Determine search direction
        double step = 0.005;
        
        T y0 = function(x0);
        if (std::abs(y0) < tol)
        {
            return true;
        }
        if (std::isnan(y0)) return false;

        // Search for boundary
        T yPlus = function(x0 + step);
        if (std::isnan(yPlus)) return false;
        int stepDir = (y0 > 0 == y0 > yPlus) ? 1 : -1;

        T x1 = x0 + step * stepDir;
        T y1;
        
        bool boundryFound = false;
        for (int i = 0; i != 500; ++i)
        {
            y1 = function(x1);
            if (std::isnan(y1)) return false;
            if (y1 > 0 != y0 > 0)
            {
                boundryFound = true;
                break;
            }
            step *= 1.02;
            x1 += step * stepDir;
        }

        if (!boundryFound)
        {
            spdlog::warn("Exceed max loop while Find other bound");
            return false;
        }

        // Narrow down root
        for (int i = 0; i != 80; ++i)
        {
            T xM = (x0 + x1) / 2;
            T yM = function(xM);
            if (std::isnan(yM)) return false;
            if (std::abs(yM) < tol)
            {
                x0 = xM;
                return true;
            }
            if (y0 > 0 == yM > 0)
            {
                x0 = xM;
                y0 = yM;
            }
            else
            {
                x1 = xM;
                y1 = yM;
            }
        }

        spdlog::warn("Exceed max loop! x0={} x1={}", x0, x1);
        return false;
    }

    struct FitResult
    {
        double beginCrv;
        double endCrv;
        double length;

        template<class Archive>
        void serialize(Archive& archive)
        {
            archive(beginCrv, endCrv, length);
        }
    };

    namespace
    {
        std::unique_ptr<odr::RoadGeometry> FitUnitSpiral(
            const double endPosAngle, const double endHdgAngle, double posPrecision, double hdgPrecision,
            int& complexityStat, const int complexityLimit)
        {
            const odr::Vec2D startPos{ 0, 0 };
            const odr::Vec2D startHdg{ 1, 0 };
            const odr::Vec2D endPos = odr::mut(UnitRadius, odr::Vec2D{ std::cos(endPosAngle), std::sin(endPosAngle) });
            const odr::Vec2D endHdg{ std::cos(endHdgAngle), std::sin(endHdgAngle) };

            auto baseArcOrLine = FitArcOrLine(startPos, startHdg, endPos);
            auto baseArc = dynamic_cast<odr::Arc*>(baseArcOrLine.get());
            if (baseArc == nullptr)
                return nullptr;

            auto baseArcEndHdg = baseArc->get_grad(baseArc->length);
            auto angleErr = odr::angle(endHdg, baseArcEndHdg);

            if (std::abs(angleErr) < 1e-4)
            {
                return baseArcOrLine;
            }

            const double startAngle = std::atan2(startHdg[1], startHdg[0]);
            const double arcLen = baseArc->length;
            const double baseCrv = baseArc->curvature;

            double startMul = angleErr > 0 ? 0.99 : 1.01;
            double endMul;
            double spiralLen = baseArc->length;

            // Bend the arc int spire

            auto endHdgErrorFunc = [=, &endMul, &spiralLen, &complexityStat](double startMul)
            {
                // Fit endMul so that spire pass through endPos
                endMul = 2 - startMul; // Initial guess

                auto endPosErrorFunc = [=, &spiralLen, &complexityStat](double endMulVar)
                {
                    auto spiralTest = odr::Spiral(0, startPos[0], startPos[1], startAngle, arcLen, baseCrv * startMul, baseCrv * endMulVar);

                    if (!FindZeroCrossing([&spiralTest, endPos, &complexityStat, complexityLimit](double s) {
                        complexityStat++;
                        if (complexityStat == complexityLimit)
                        {
                            return std::nan("2");
                        }
                        return spiralTest.get_signed_error(endPos, s);
                        }, spiralLen, posPrecision))
                    {
                        spdlog::warn("Inner: Cannot find closest S!");
                        return std::nan("1");
                    }

                        double closeS = spiralLen;
                        auto closeP = spiralTest.get_xy(closeS);
                        auto pToTarget = odr::sub(endPos, closeP);
                        auto grad = odr::normalize(spiralTest.get_grad(closeS));
                        auto tan = odr::Vec2D{ grad[1], -grad[0] };
                        auto err = odr::dot(pToTarget, tan);
                        complexityStat++;
                        if (complexityStat == complexityLimit)
                        {
                            return std::nan("2");
                        }
                        return err; // Signed distance for Newton
                };

                bool posFitSuccess = FindZeroCrossing(endPosErrorFunc, endMul, posPrecision);

                if (!posFitSuccess)
                {
                    spdlog::warn("Cannot approx spiral to end pos ({}, {})", endPosAngle / M_PI * 180, endHdgAngle / M_PI * 180);
                    return std::nan("0"); // Mark solution invalid; Exit outer loop immediately
                }

                auto spiralTest = odr::Spiral(0, startPos[0], startPos[1], startAngle, arcLen, baseCrv * startMul, baseCrv * endMul);

                if (!FindZeroCrossing([&spiralTest, endPos, &complexityStat, complexityLimit](double s) {
                    complexityStat++;
                    if (complexityStat == complexityLimit)
                    {
                        return std::nan("2");
                    }
                    return spiralTest.get_signed_error(endPos, s);
                    }, spiralLen, posPrecision))
                {
                    spdlog::warn("Outer: Cannot find closest S!");
                    return std::nan("1");
                }

                    double closeS = spiralLen;
                    auto candAngle = spiralTest.get_grad(closeS);
                    auto candErr = odr::angle(endHdg, candAngle);
                    return candErr;
            };

            // Fit startMul so that spire pass through endPos at endHdg
            bool dirFitSuccess = FindZeroCrossing(endHdgErrorFunc, startMul, hdgPrecision);

            if (dirFitSuccess && endMul != 0)
            {
                auto startCrv = baseCrv * startMul;
                auto endCrvNL = baseCrv * endMul;
                auto resultNoLength = odr::Spiral(0, startPos[0], startPos[1], startAngle, arcLen, startCrv, endCrvNL);

                if (!FindZeroCrossing([&resultNoLength, endPos](double s) {
                    return resultNoLength.get_signed_error(endPos, s);
                    },
                    spiralLen, posPrecision))
                {
                    spdlog::warn("Result: Cannot find closest S!");
                    return nullptr;
                }

                    auto cDot = (endCrvNL - startCrv) / arcLen;
                    double length = spiralLen;
                    auto endCrv = startCrv + cDot * length;
                    auto result = std::make_unique<odr::Spiral>(0, startPos[0], startPos[1], startAngle,
                        length, startCrv, endCrv);

                    auto endP = result->get_xy(length);
                    auto endH = result->get_grad(length);
                    auto pError = odr::euclDistance(endP, endPos);
                    auto hError = odr::angle(endH, endHdg);

                    spdlog::trace("[ANS] start mul = {}, end mul = {}, len = {} => pErr = {}, hErr = {}",
                        startMul, endMul, length, pError, hError);

// Don't need self-check in normal run, as stepwise verification will cover this.
#ifdef G_TEST
                    if (pError > SpiralPosPrecision || std::abs(hError) > SpiralHdgPrecision)
                    {
                        spdlog::error("[Fail check] {},{}", pError, hError);
                        return nullptr;
                    }
#endif
                    return result;
            }
            return nullptr;
        }
    }

    const std::map<int, std::pair<int, int>> posAngleCombo =
    {
        {5, {5, 60}},
        {10, {5, 70}},
        {15, {5, 110}},
        {20, {5, 130}},
        {25, {5, 130}},
        {30, {5, 130}},
        {35, {5, 135}},
        {40, {5, 140}},
        {45, {5, 140}},
        {50, {5, 140}},
        {55, {5, 140}},
        {60, {5, 140}},
        {65, {5, 140}},
        {70, {5, 140}},
        {75, {5, 140}},
        {80, {5, 140}},
        {85, {5, 140}},
        {90, {5, 140}},
        {95, {5, 140}},
        {100, {5, 140}},
        {105, {5, 140}},
        {110, {5, 140}},
        {115, {5, 140}},
        {120, {5, 145}},
        {125, {5, 145}},
        {130, {5, 145}},
        {135, {10, 145}},
        {140, {20, 145}},
        {145, {35, 145}},
        {150, {40, 145}},
        {155, {50, 145}},
        {160, {60, 95}},
    };

    constexpr double degToRad = M_PI / 180;

    std::unique_ptr<odr::RoadGeometry> FitSpiral(const odr::Vec2D& startPos, const odr::Vec2D& startHdg,
        const odr::Vec2D& endPos, const odr::Vec2D& endHdg)
    {
        auto localP = odr::toLocal(endPos, startPos, startHdg);
        auto localA = odr::angle(startHdg, endHdg);
        auto localPAngle = std::atan2(localP[1], localP[0]);
        int outputCrvMul = 1;
        auto turnAngle = odr::angle(odr::sub(endPos, startPos), endHdg);

        if (localPAngle < 0)
        {
            outputCrvMul = -1; // Flip against X axis
            localPAngle *= -1;
            localA *= -1;
            turnAngle *= -1;
        }

        // Check if input is within good range where solution is likely to exist
        double localPAngleDeg = localPAngle / degToRad;
        double turnAngleDeg = turnAngle / degToRad;
        int pLow = std::floor(localPAngleDeg / 5) * 5;
        int pHi = std::ceil(localPAngleDeg / 5) * 5;
        if (posAngleCombo.find(pLow) == posAngleCombo.end() || posAngleCombo.find(pHi) == posAngleCombo.end())
        {
            spdlog::trace("Invalid input: end pos out of range: {}-{}", pLow, pHi);
            return nullptr;
        }

        auto pLowRange = posAngleCombo.at(pLow);
        auto pHiRange = posAngleCombo.at(pHi);
        if (!(pLowRange.first <= turnAngleDeg && turnAngleDeg <= pLowRange.second &&
              pHiRange.first <= turnAngleDeg && turnAngleDeg <= pHiRange.second))
        {
            spdlog::trace("Invalid input: end angle out of range");
            return nullptr;
        }

        auto lengthBoost = odr::euclDistance(startPos, endPos) / UnitRadius;
        if (lengthBoost > MaximumLengthBoost)
        {
            spdlog::info("Invalid input: crv is too long to fit within default precision.");
            return nullptr;
        }

        // Do the fitting, but constrain complexity to avoid UI freeze
        int complexityStat = 0;
        auto fitResult = FitUnitSpiral(localPAngle, localA, SpiralPosPrecision * 0.7 / lengthBoost, SpiralHdgPrecision * 0.7, complexityStat, 400000);
        if (fitResult != nullptr)
        {
            auto globalA = std::atan2(startHdg[1], startHdg[0]);
            
            auto fitArc = dynamic_cast<odr::Arc*>(fitResult.get());

            if (fitArc != nullptr)
            {
                return std::make_unique<odr::Arc>(0, startPos[0], startPos[1], globalA,
                    fitArc->length * lengthBoost, fitArc->curvature / lengthBoost * outputCrvMul);
            }
            auto fitSpiral = dynamic_cast<odr::Spiral*>(fitResult.get());
            return std::make_unique<odr::Spiral>(0, startPos[0], startPos[1], globalA,
                fitSpiral->length * lengthBoost, 
                fitSpiral->curv_start / lengthBoost * outputCrvMul, fitSpiral->curv_end / lengthBoost * outputCrvMul);
        }

        spdlog::warn("Valid input but exceeded complexity constraint");
        return nullptr;
    }

    std::unique_ptr<odr::RoadGeometry> FitArcOrLine(const odr::Vec2D& startPos,
        const odr::Vec2D& startHdg, const odr::Vec2D& endPos)
    {
        Point_2 <Kernel> p1(startPos[0], startPos[1]);
        Point_2 <Kernel> p2(endPos[0], endPos[1]);

        double startAngle = std::atan2(startHdg[1], startHdg[0]);
        auto start2End = odr::sub(endPos, startPos);
        start2End = odr::normalize(start2End);

        Direction_2< Kernel > dir1(startHdg[0], startHdg[1]);
        Direction_2< Kernel> dir2(start2End[0], start2End[1]);
        Ray_2< Kernel > ray1(p1, dir1);
        Ray_2< Kernel > ray2(p2, -dir2);

        if (squared_distance(ray1, p2) < std::pow(1e-4, 2) && squared_distance(ray2, p1) < std::pow(1e-4, 2))
        {
            // collinear case
            return std::make_unique<odr::Line>(0, startPos[0], startPos[1], startAngle, odr::euclDistance(startPos, endPos));
        }

        Direction_2<Kernel> tan1(-startHdg[1], startHdg[0]);
        Line_2 <Kernel> line1(p1, tan1);

        Direction_2<Kernel> tan2(-start2End[1], start2End[0]);
        auto mid = odr::mut(0.5, odr::add(startPos, endPos));
        Point_2 <Kernel> pMid(mid[0], mid[1]);
        Line_2 <Kernel> line2(pMid, tan2);

        Object result = intersection(line1, line2);
        const CGAL::Point_2<Kernel>* po = object_cast<Point_2<Kernel>>(&result);

        if (po == nullptr)
        {
            spdlog::trace("No intersection found for circle center");
            return nullptr;
        }

        auto r = CGAL::squared_distance(p1, *po);
        r = std::sqrt(r);
        odr::Vec2D O{ po->x(), po->y() };
        auto O2Start = odr::sub(startPos, O);
        bool cw = odr::crossProduct(startHdg, O2Start) > 0;

        auto O2End = odr::sub(endPos, O);

        auto arcAngle = odr::angle(O2Start, O2End);
        if (cw == (arcAngle > 0))
        {
            arcAngle = M_PI * 2 - std::abs(arcAngle);
        }
        else
        {
            arcAngle = std::abs(arcAngle);
        }
        const auto arcLen = arcAngle * r;

        auto baseCrv = (cw ? -1.0 : 1.0) / r;
        return std::make_unique<odr::Arc>(0, startPos[0], startPos[1], startAngle, arcLen, baseCrv);
    }

#ifdef G_TEST
    void TestSpiralFitting()
    {
        std::vector<double> timeStats;
        std::vector<int> complexityStats;

        for (auto posAndAngle : posAngleCombo)
        {
            int posAngle = posAndAngle.first;

            int hdgBegin = posAndAngle.second.first;
            int hdgEnd = posAndAngle.second.second;

            std::map<int, FitResult> hdgToRes;
            for (int hdg = hdgBegin; hdg <= hdgEnd; hdg += 5)
            {
                int hdgAngle = (posAngle + hdg);
                int complexity = 0;
                auto t_start = high_resolution_clock::now();
                auto fitGeo = LM::FitUnitSpiral(
                    posAngle * degToRad, hdgAngle * degToRad,
                    SpiralPosPrecision * 0.7 / MaximumLengthBoost, SpiralHdgPrecision * 0.7 / MaximumLengthBoost, complexity);
                auto t_end = high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(t_end - t_start).count();
                timeStats.push_back(static_cast<double>(duration) / 1000000);
                complexityStats.push_back(complexity);

                EXPECT_TRUE(fitGeo != nullptr) << "No ans given";
                if (complexity > 300000)
                {
                    spdlog::info("High complexity at angle {} hdg {}", posAngle, hdg);
                }

                FitResult res;
                auto fitArc = dynamic_cast<odr::Arc*>(fitGeo.get());
                auto fitSpiral = dynamic_cast<odr::Spiral*>(fitGeo.get());
                if (fitArc != nullptr)
                {
                    res = FitResult{ fitArc->curvature, fitArc->curvature, fitArc->length };
                }
                else if (fitSpiral != nullptr)
                {
                    res = FitResult{ fitSpiral->curv_start, fitSpiral->curv_end, fitSpiral->length };
                }
                else
                {
                    FAIL();
                }
                hdgToRes.emplace(hdg, res);
            }
        }
        std::sort(timeStats.begin(), timeStats.end());
        std::sort(complexityStats.begin(), complexityStats.end());

        int percentile = static_cast<double>(timeStats.size()) * 0.95;
        spdlog::info("95 Fitting micros takes {}s; complexity = {}",
            timeStats[percentile],
            complexityStats[percentile]);
    }
#endif
}