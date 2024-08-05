#pragma once

#include "Geometries/RoadGeometry.h"

namespace RoadRunner
{
    // l1 --> conn --> l2
    std::unique_ptr<odr::RoadGeometry> ConnectLines(const odr::Vec2D& startPos, const odr::Vec2D& startHdg,
        const odr::Vec2D& endPos, const odr::Vec2D& endHdg);

    /*Algorithm designed for this precesion for R < 500*/
    const double SpiralPosPrecision = 1e-3;
    const double SpiralHdgPrecision = 1e-3;
    namespace
    {
        const double UnitRadius = 10;
        const double MaximumLengthBoost = 50;
    }

    std::unique_ptr<odr::RoadGeometry> FitUnitSpiral(
        const double endPosAngle, const double endHdgAngle, 
        double posPrecision, double hdgPrecision,
        int& complexityStat, const int complexityLimit = 0);

    void WriteFitTable();

    std::unique_ptr<odr::RoadGeometry> FitSpiral(
        const odr::Vec2D& startPos, const odr::Vec2D& startHdg,
        const odr::Vec2D& endPos, const odr::Vec2D& endHdg);
}