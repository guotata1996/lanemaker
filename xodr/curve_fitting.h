#pragma once

#include "Geometries/RoadGeometry.h"
#include "Geometries/Arc.h"

namespace LM
{
    std::unique_ptr<odr::RoadGeometry> ConnectRays(const odr::Vec2D& startPos, const odr::Vec2D& startHdg,
        const odr::Vec2D& endPos, const odr::Vec2D& endHdg);

    std::unique_ptr<odr::RoadGeometry> FitParamPoly(const odr::Vec2D& startPos, const odr::Vec2D& startHdg,
        const odr::Vec2D& endPos, const odr::Vec2D& endHdg);

    namespace
    {
        /*Algorithm designed for this precesion for R < 500*/
        const double SpiralPosPrecision = 1e-3;
        const double SpiralHdgPrecision = 1e-3;
        const double UnitRadius = 10;
        const double MaximumLengthBoost = 50;

        std::unique_ptr<odr::RoadGeometry> FitUnitSpiral(
            const double endPosAngle, const double endHdgAngle,
            double posPrecision, double hdgPrecision,
            int& complexityStat, const int complexityLimit = 0);
    }

    std::unique_ptr<odr::RoadGeometry> FitSpiral(
        const odr::Vec2D& startPos, const odr::Vec2D& startHdg,
        const odr::Vec2D& endPos, const odr::Vec2D& endHdg);

#ifdef G_TEST
    void TestSpiralFitting();
#endif

    std::unique_ptr<odr::RoadGeometry> FitArcOrLine(const odr::Vec2D& startPos,
        const odr::Vec2D& startHdg, const odr::Vec2D& endPos);
}