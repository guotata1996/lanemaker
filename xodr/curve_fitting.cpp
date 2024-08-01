#pragma once
#include "curve_fitting.h"

#include <CGAL/intersections.h>
#include <CGAL/Cartesian.h>

#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
#include "Geometries/Arc.h"
#include "Geometries/Spiral.h"
#include "CubicBezier.hpp"
#include "spdlog/spdlog.h"

using namespace CGAL;

typedef Cartesian<double>  Kernel;

namespace RoadRunner
{
    // l1 --> conn --> l2
    std::unique_ptr<odr::RoadGeometry> ConnectLines(const odr::Vec2D& startPos, const odr::Vec2D& startHdg,
        const odr::Vec2D& endPos, const odr::Vec2D& endHdg)
    {
        double hdg = std::atan2(startHdg[1], startHdg[0]);
        Point_2 <Kernel> p1(startPos[0], startPos[1]), p2(endPos[0], endPos[1]);
        Direction_2< Kernel > dir1(startHdg[0], startHdg[1]);
        Direction_2< Kernel> dir2(endHdg[0], endHdg[1]);
        Ray_2< Kernel > ray1(p1, dir1);
        Ray_2< Kernel > ray2(p2, -dir2);

        if (squared_distance(ray1, p2) < std::pow(1e-4, 2) && squared_distance(ray2, p1) < std::pow(1e-4, 2))
        {
            // collinear case
            return std::make_unique<odr::Line>(0, startPos[0], startPos[1], hdg, odr::euclDistance(startPos, endPos));
        }

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
    bool Newton(F xToError, T& x, double tol = 1e-4, bool verbose = true)
    {
        const double slopeProb = 2e-4; // TODO: should be relative to magnitude of delta_x
        for (int i = 0; i != 30; ++i)
        {
            double errA = xToError(x);
            if (verbose && i > 10)
            {
                spdlog::info("Iter {}: x={} err={}", i, x, errA);
            }

            if (std::abs(errA) < tol)
            {
                return true;
            }

            double errB = xToError(x + slopeProb);
            if (std::abs(errA - errB) < slopeProb * 1e-3)
            {
                return false;
            }
            double slope = (errB - errA) / slopeProb;
            x -= errA / slope * 0.8;
        }
        return false;
    }

    std::unique_ptr<odr::RoadGeometry> FitSpiral(const odr::Vec2D& startPos, const odr::Vec2D& startHdg,
        const odr::Vec2D& endPos, const odr::Vec2D& endHdg)
    {
        Direction_2<Kernel> dir1(-startHdg[1], startHdg[0]);
        Point_2 <Kernel> p1(startPos[0], startPos[1]);
        Line_2 <Kernel> line1(p1, dir1);

        auto start2End = odr::sub(endPos, startPos);
        start2End = odr::normalize(start2End);
        Direction_2<Kernel> dir2(-start2End[1], start2End[0]);
        auto mid = odr::mut(0.5, odr::add(startPos, endPos));
        Point_2 <Kernel> p2(mid[0], mid[1]);
        Line_2 <Kernel> line2(p2, dir2);

        Object result = intersection(line1, line2);
        const CGAL::Point_2<Kernel>* po = object_cast<Point_2<Kernel>>(&result);

        if (po == nullptr)
        {
            spdlog::info("No intersection found for circle center");
            return;
        }

        auto r = CGAL::squared_distance(p1, *po);
        r = std::sqrt(r);
        odr::Vec2D O{ po->x(), po->y() };
        auto O2Start = odr::sub(startPos, O);
        bool cw = odr::crossProduct(startHdg, O2Start) > 0;

        auto O2End = odr::sub(endPos, O);
        odr::Vec2D arcEndHdg{ O2End[1], -O2End[0] };
        if (!cw)
        {
            arcEndHdg = odr::negate(arcEndHdg);
        }
        auto angleErr = odr::angle(endHdg, arcEndHdg);
        
        auto arcAngle = odr::angle(O2Start, O2End);
        if (cw == (arcAngle > 0))
        {
            arcAngle = M_PI * 2 - std::abs(arcAngle);
        }
        else
        {
            arcAngle = std::abs(arcAngle);
        }
        auto arcLen = arcAngle * r;
        
        spdlog::info("Start from {} circle centered at ({},{}) r={} Angle={} => end hdg err={}",
            cw ? "CW" : "CCW",
            po->x(), po->y(), r, arcAngle, angleErr);

        auto baseCrv = (cw ? -1.0 : 1.0) / r;
        double startAngle = std::atan2(startHdg[1], startHdg[0]);
        

        if (std::abs(angleErr) < 1e-4)
        {
            return std::make_unique<odr::Arc>(0, startPos[0], startPos[1], startAngle, arcLen, baseCrv);
        }

        // Bend the arc int spire
        double startMul = angleErr > 0 ? 0.99 : 1.01;
        double endMul;

        auto endHdgErrorFunc = [=, &endMul](double startMul)
        {
            // Fit endMul so that spire pass through endPos
            endMul = 2 - startMul; // Initial guess

            auto endPosErrorFunc = [=](double x)
            {
                // TODO: treat special case: start crv == end crv
                auto spiralTest = odr::Spiral(0, startPos[0], startPos[1], startAngle, 1, baseCrv * startMul, baseCrv * x);
                double closeS = spiralTest.get_closest_s_to(endPos, arcLen);
                //spdlog::info(" {} -> Close S = {};", arcLen, closeS);
                auto closeP = spiralTest.get_xy(closeS);
                return odr::euclDistance(closeP, endPos);
            };

            bool posFitSuccess = Newton(endPosErrorFunc, endMul);

            if (!posFitSuccess)
            {
                spdlog::warn("Cannot approx spiral to end pos!");
                endMul = 0; // Mark solution invalid
                return 0.0; // Exit outer loop immediately
            }

            auto spiralTest = odr::Spiral(0, startPos[0], startPos[1], startAngle, 1, baseCrv * startMul, baseCrv * endMul);
            double closeS = spiralTest.get_closest_s_to(endPos, arcLen);
            auto candAngle = spiralTest.get_grad(closeS);
            auto candErr = odr::angle(endHdg, candAngle);
            //spdlog::info("  OuterLoop: start mul={} end mul={} angle diff = {}", startMul, endMul, candErr);
            return candErr;
        };

        // Fit startMul so that spire pass through endPos at endHdg
        bool dirFitSuccess = Newton(endHdgErrorFunc, startMul);

        if (dirFitSuccess && endMul != 0)
        {
            auto fitResult = odr::Spiral(0, startPos[0], startPos[1], startAngle, 1, baseCrv * startMul, baseCrv * endMul);
            double endS = fitResult.get_closest_s_to(endPos, arcLen);
            auto endP = fitResult.get_xy(endS);
            auto endH = fitResult.get_grad(endS);
            auto pError = odr::euclDistance(endP, endPos);
            auto hError = odr::angle(endH, endHdg);

            spdlog::info("Ans: start mul = {}, end mul = {}, len = {} => pErr = {}, hErr = {}", 
                startMul, endMul, endS, pError, hError);
            return std::make_unique<odr::Spiral>(0, startPos[0], startPos[1], startAngle, endS, baseCrv * startMul, baseCrv * endMul);
        }
        return nullptr;
    }
}