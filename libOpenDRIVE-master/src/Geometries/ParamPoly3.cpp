#include "Geometries/ParamPoly3.h"
#include "Geometries/RoadGeometry.h"
#include "Math.hpp"

#include <array>
#include <cmath>
#include <map>

namespace odr
{

ParamPoly3::ParamPoly3(double s0,
                       double x0,
                       double y0,
                       double hdg0,
                       double length,
                       double aU,
                       double bU,
                       double cU,
                       double dU,
                       double aV,
                       double bV,
                       double cV,
                       double dV,
                       bool   pRange_normalized) :
    RoadGeometry(s0, x0, y0, hdg0, length, GeometryType_ParamPoly3),
    aU(aU), bU(bU), cU(cU), dU(dU), aV(aV), bV(bV), cV(cV), dV(dV), pRange_normalized(pRange_normalized)
{
    if (!pRange_normalized)
    {
        this->bU = bU * length;
        this->bV = bV * length;
        this->cU = cU * length * length;
        this->cV = cV * length * length;
        this->dU = dU * length * length * length;
        this->dV = dV * length * length * length;
    }

    const std::array<Vec2D, 4> coefficients = {{{this->aU, this->aV}, {this->bU, this->bV}, {this->cU, this->cV}, {this->dU, this->dV}}};
    this->cubic_bezier = CubicBezier2D(CubicBezier2D::get_control_points(coefficients));

    this->cubic_bezier.arclen_t[length] = 1.0;
    this->cubic_bezier.valid_length = length;
}

std::unique_ptr<RoadGeometry> ParamPoly3::clone() const { return std::make_unique<ParamPoly3>(*this); }

Vec2D ParamPoly3::get_xy(double s) const
{
    const double p = this->cubic_bezier.get_t(s - s0);
    const Vec2D  pt = this->cubic_bezier.get(p);

    const double xt = (std::cos(hdg0) * pt[0]) - (std::sin(hdg0) * pt[1]) + x0;
    const double yt = (std::sin(hdg0) * pt[0]) + (std::cos(hdg0) * pt[1]) + y0;

    return Vec2D{xt, yt};
}

Vec2D ParamPoly3::get_grad(double s) const
{
    const double p = this->cubic_bezier.get_t(s - s0);
    const Vec2D  dxy = this->cubic_bezier.get_grad(p);

    const double h1 = std::cos(hdg0);
    const double h2 = std::sin(hdg0);
    const double dx = h1 * dxy[0] - h2 * dxy[1];
    const double dy = h2 * dxy[0] + h1 * dxy[1];

    return {{dx, dy}};
}

std::set<double> ParamPoly3::approximate_linear(double eps) const
{
    std::set<double> p_vals = this->cubic_bezier.approximate_linear(eps);

    std::set<double> s_vals;
    for (const double& p : p_vals)
        s_vals.insert(p * length + s0);

    return s_vals;
}

void ParamPoly3::reverse() 
{ 
    decltype(cubic_bezier.control_points) revCtrlPoints;
    odr::Vec2D                            oldOrigin = get_xy(0);
    odr::Vec2D                            oldHdg = odr::normalize(get_grad(0));
    odr::Vec2D                            newOrigin = get_xy(length);
    odr::Vec2D                            newHdg = odr::normalize(odr::negate(get_grad(length)));

    for (int i = 0; i != 4; ++i)
    {
        auto globalCtrl = odr::toGlobal(cubic_bezier.control_points[i], oldOrigin, oldHdg);
        revCtrlPoints[3 - i] = odr::toLocal(globalCtrl, newOrigin, newHdg);
    }

    x0 = newOrigin.at(0);
    y0 = newOrigin.at(1);
    hdg0 = std::atan2(newHdg[1], newHdg[0]);
    if (hdg0 >= 2 * M_PI)
    {
        hdg0 -= 2 * M_PI;
    }

    auto coefficients = odr::CubicBezier2D::get_coefficients(revCtrlPoints);
    aU = coefficients[0][0];
    bU = coefficients[1][0];
    cU = coefficients[2][0];
    dU = coefficients[3][0];
    aV = coefficients[0][1];
    bV = coefficients[1][1];
    cV = coefficients[2][1];
    dV = coefficients[3][1];

    this->cubic_bezier = CubicBezier2D(revCtrlPoints);
    this->cubic_bezier.arclen_t[length] = 1.0;
    this->cubic_bezier.valid_length = length;

    //double au1 = aU + bU + cU + dU;
    //double bu1 = -bU - 2 * cU - 3 * dU;
    //double cu1 = cU + 3 * dU;
    //double du1 = -dU;

    //double av1 = aV + bV + cV + dV;
    //double bv1 = -bV - 2 * cV - 3 * dV;
    //double cv1 = cV + 3 * dV;
    //double dv1 = -dV;

    //aU = au1; bU = bu1; cU = cu1; dU = du1;
    //aV = av1; bV = bv1; cV = cv1; dV = dv1;

    //const std::array<Vec2D, 4> coefficients = {{{this->aU, this->aV}, {this->bU, this->bV}, {this->cU, this->cV}, {this->dU, this->dV}}};
    //this->cubic_bezier = CubicBezier2D(CubicBezier2D::get_control_points(coefficients));
    //this->cubic_bezier.arclen_t[length] = 1.0;
    //this->cubic_bezier.valid_length = length;
}
} // namespace odr
