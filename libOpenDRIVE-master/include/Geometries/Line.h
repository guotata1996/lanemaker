#pragma once
#include "Math.hpp"
#include "RoadGeometry.h"

#include <memory>
#include <set>

namespace odr
{

struct Line : public RoadGeometry
{
    Line(double s0, double x0, double y0, double hdg0, double length);

    Line(double s0, Vec2D a, Vec2D b);

    std::unique_ptr<RoadGeometry> clone() const override;

    Vec2D get_xy(double s) const override;
    Vec2D get_grad(double s) const override;

    std::set<double> approximate_linear(double eps) const override;
};

} // namespace odr