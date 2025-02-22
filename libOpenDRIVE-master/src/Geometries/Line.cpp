#include "Geometries/Line.h"
#include "Geometries/RoadGeometry.h"
#include "Math.hpp"

#include <cmath>

namespace odr
{

Line::Line(double s0, double x0, double y0, double hdg0, double length) : RoadGeometry(s0, x0, y0, hdg0, length, GeometryType_Line) {}

Line::Line(double s0, Vec2D a, Vec2D b) :
    RoadGeometry(s0, a[0], a[1], 0, 0, GeometryType_Line) 
{ 
    hdg0 = std::atan2(b[1] - a[1], b[0] - a[0]);
    length = odr::euclDistance(a, b);
}

std::unique_ptr<RoadGeometry> Line::clone() const { return std::make_unique<Line>(*this); }

Vec2D Line::get_xy(double s) const
{
    const double x = (std::cos(hdg0) * (s - s0)) + x0;
    const double y = (std::sin(hdg0) * (s - s0)) + y0;
    return Vec2D{x, y};
}

Vec2D Line::get_grad(double s) const { return {{std::cos(hdg0), std::sin(hdg0)}}; }

std::set<double> Line::approximate_linear(double eps) const { return {s0, s0 + length}; }

} // namespace odr
