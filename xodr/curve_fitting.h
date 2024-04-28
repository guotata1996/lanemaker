#pragma once

#include "Geometries/RoadGeometry.h"

namespace RoadRunner
{
    // l1 --> conn --> l2
    std::unique_ptr<odr::RoadGeometry> ConnectLines(const odr::Vec2D& startPos, const odr::Vec2D& startHdg,
        const odr::Vec2D& endPos, const odr::Vec2D& endHdg);
}