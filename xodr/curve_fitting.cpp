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
    bool FindZeroCrossing(F function, T& x0, double tol = 7e-5)
    {
        // Determine search direction
        double step = 0.001;
        
        T y0 = function(x0);
        if (std::abs(y0) < tol)
        {
            return true;
        }

        // Search for boundary
        T yPlus = function(x0 + step);
        int stepDir = (y0 > 0 == y0 > yPlus) ? 1 : -1;

        T x1 = x0 + step * stepDir;
        T y1;
        
        bool boundryFound = false;
        for (int i = 0; i != 300; ++i)
        {
            y1 = function(x1);
            if (y1 > 0 != y0 > 0)
            {
                boundryFound = true;
                break;
            }
            step *= 1.03;
            x1 += step * stepDir;
        }

        if (!boundryFound)
        {
            spdlog::info("Exceed max loop! while Find other bound");
            return false;
        }

        // Narrow down root
        for (int i = 0; i != 80; ++i)
        {
            T xM = (x0 + x1) / 2;
            T yM = function(xM);
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

        spdlog::info("Exceed max loop! x0={} x1={}", x0, x1);
        return false;
    }

    std::unique_ptr<odr::RoadGeometry> FitSpiral(const odr::Vec2D& startPos, const odr::Vec2D& startHdg,
        const odr::Vec2D& endPos, const odr::Vec2D& endHdg)
    {
        /*
        auto base = odr::Spiral(0, 0, 0, 0, 1.5, 0.91, 0.72);
        auto baseP = base.get_grad(base.length);
        spdlog::info("Base pt: {},{}", baseP[0], baseP[1]);
        auto ext = odr::Spiral(0, 0, 0, 0, 15, 0.091, 0.072);
        auto extP = ext.get_grad(ext.length);
        spdlog::info("Ext pt: {},{}", extP[0], extP[1]);*/
        
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
            return nullptr;
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
        const auto arcLen = arcAngle * r;
        
        /*spdlog::info("Start from {} circle centered at ({},{}) r={} Angle={} => end hdg err={}",
            cw ? "CW" : "CCW",
            po->x(), po->y(), r, arcAngle, angleErr);*/

        auto baseCrv = (cw ? -1.0 : 1.0) / r;
        double startAngle = std::atan2(startHdg[1], startHdg[0]);
        

        if (std::abs(angleErr) < 1e-4)
        {
            return std::make_unique<odr::Arc>(0, startPos[0], startPos[1], startAngle, arcLen, baseCrv);
        }

        // Bend the arc int spire
        double startMul = angleErr > 0 ? 0.99 : 1.01;
        double endMul;
        auto spiralLen = arcLen;

        auto endHdgErrorFunc = [=, &endMul, &spiralLen](double startMul)
        {
            // Fit endMul so that spire pass through endPos
            endMul = 2 - startMul; // Initial guess

            auto endPosErrorFunc = [=, &spiralLen](double endMulVar)
            {
                auto spiralTest = odr::Spiral(0, startPos[0], startPos[1], startAngle, arcLen, baseCrv * startMul, baseCrv * endMulVar);

                if (!FindZeroCrossing([&spiralTest, endPos](double s){
                        return spiralTest.get_signed_error(endPos, s);
                    }, spiralLen))
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
                return err; // Signed distance for Newton
            };

            bool posFitSuccess = FindZeroCrossing(endPosErrorFunc, endMul);

            if (!posFitSuccess)
            {
                spdlog::warn("Cannot approx spiral to end pos at start mul = {}", startMul);
                return std::nan("0"); // Mark solution invalid; Exit outer loop immediately
            }

            auto spiralTest = odr::Spiral(0, startPos[0], startPos[1], startAngle, arcLen, baseCrv * startMul, baseCrv * endMul);

            if (!FindZeroCrossing([&spiralTest, endPos](double s){
                    return spiralTest.get_signed_error(endPos, s);
                }, spiralLen))
            {
                spdlog::warn("Outer: Cannot find closest S!");
                return std::nan("1");
            }

            double closeS = spiralLen;
            auto candAngle = spiralTest.get_grad(closeS);
            auto candErr = odr::angle(endHdg, candAngle);
            //spdlog::info("  OuterLoop: start mul={} end mul={} angle diff = {}", startMul, endMul, candErr);
            return candErr;
        };

        // Fit startMul so that spire pass through endPos at endHdg
        bool dirFitSuccess = FindZeroCrossing(endHdgErrorFunc, startMul, 1e-4);

        if (dirFitSuccess && endMul != 0)
        {
            auto startCrv = baseCrv * startMul;
            auto endCrvNL = baseCrv * endMul;
            auto resultNoLength = odr::Spiral(0, startPos[0], startPos[1], startAngle, arcLen, startCrv, endCrvNL);

            if (!FindZeroCrossing([&resultNoLength, endPos](double s){
                    return resultNoLength.get_signed_error(endPos, s);
                }, 
                spiralLen))
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

            if (pError > 1e-4 || std::abs(hError) > 1e-4)
            {
                spdlog::error("[Fail check] {},{}", pError, hError);
                return nullptr;
            }

            return result;
        }
        return nullptr;
    }
}