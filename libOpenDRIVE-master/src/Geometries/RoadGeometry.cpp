#include "Geometries/RoadGeometry.h"
#include <cassert>

namespace odr
{

RoadGeometry::RoadGeometry(double s0, double x0, double y0, double hdg0, double length, GeometryType type) :
    s0(s0), x0(x0), y0(y0), hdg0(hdg0), length(length), type(type)
{
}

void RoadGeometry::reverse() 
{
    const auto pos_end = get_xy(s0 + length);
    const auto hdg_end = get_grad(s0 + length);
    hdg0 = std::atan2(hdg_end[1], hdg_end[0]) + M_PI;
    if (hdg0 >= M_PI)
    {
        hdg0 -= 2 * M_PI;
    }
    x0 = pos_end[0];
    y0 = pos_end[1];
}

void RoadGeometry::rebase(double s) {
    assert(0 <= s);
    assert(s < length);
    auto newStart = get_xy(s0 + s);
    auto newgrad = get_grad(s0 + s);
    x0 = newStart[0];
    y0 = newStart[1];
    hdg0 = std::atan2(newgrad[1], newgrad[0]);
    length -= s;
}

void RoadGeometry::trim(double l)
{ 
    assert(0 < l);
    assert(l <= length);
    length = l; 
}

Vec2D RoadGeometry::get_end() 
{ 
    return get_xy(length); 
}
} // namespace odr
