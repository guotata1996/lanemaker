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
    odr::Vec2D                            oldOrigin = get_xy(s0);
    odr::Vec2D                            oldHdg = odr::normalize(get_grad(s0));
    odr::Vec2D                            newOrigin = get_xy(s0 + length);
    odr::Vec2D                            newHdg = odr::normalize(odr::negate(get_grad(s0 + length)));

    for (int i = 0; i != 4; ++i)
    {
        auto globalCtrl = odr::toGlobal(cubic_bezier.control_points[i], oldOrigin, oldHdg);
        revCtrlPoints[3 - i] = odr::toLocal(globalCtrl, newOrigin, newHdg);
    }

    RoadGeometry::reverse();

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
}

void ParamPoly3::trim(double l) 
{ 
    RoadGeometry::trim(l);

    // https://stackoverflow.com/questions/18655135/divide-bezier-curve-into-two-equal-halves
    auto A = this->cubic_bezier.control_points[0];
    auto B = this->cubic_bezier.control_points[1];
    auto C = this->cubic_bezier.control_points[2];

    double cut_t = this->cubic_bezier.get_t(l);
    auto   E = odr::lerp(A, B, cut_t);
    auto   F = odr::lerp(B, C, cut_t);
    auto   H = odr::lerp(E, F, cut_t);
    auto   K = this->cubic_bezier.get(cut_t);

    decltype(cubic_bezier.control_points) newCtrlPoints = {A, E, H, K};

    auto coefficients = odr::CubicBezier2D::get_coefficients(newCtrlPoints);
    aU = coefficients[0][0];
    bU = coefficients[1][0];
    cU = coefficients[2][0];
    dU = coefficients[3][0];
    aV = coefficients[0][1];
    bV = coefficients[1][1];
    cV = coefficients[2][1];
    dV = coefficients[3][1];

    this->cubic_bezier = CubicBezier2D(newCtrlPoints);
    this->cubic_bezier.arclen_t[length] = 1.0;
    this->cubic_bezier.valid_length = length;
}

void ParamPoly3::rebase(double s) 
{
    auto oldOrigin = get_xy(s0);
    auto oldHdg = odr::normalize(get_grad(s0));
    auto newOrigin = get_xy(s0 + s);
    auto newHdg = odr::normalize(get_grad(s0 + s));

    RoadGeometry::rebase(s);

    // https://stackoverflow.com/questions/18655135/divide-bezier-curve-into-two-equal-halves
    auto B = this->cubic_bezier.control_points[1];
    auto C = this->cubic_bezier.control_points[2];
    auto D = this->cubic_bezier.control_points[3];

    double cut_t = this->cubic_bezier.get_t(s);
    auto   F = odr::lerp(B, C, cut_t);
    auto   G = odr::lerp(C, D, cut_t);
    auto   J = odr::lerp(F, G, cut_t);
    auto   K = this->cubic_bezier.get(cut_t);

    decltype(cubic_bezier.control_points) newCtrlPoints = {K, J, G, D};
    for (int i = 0; i != 4; ++i) 
    {
        auto globalCtrl = odr::toGlobal(newCtrlPoints[i], oldOrigin, oldHdg);
        newCtrlPoints[i] = odr::toLocal(globalCtrl, newOrigin, newHdg);
    }
    auto coefficients = odr::CubicBezier2D::get_coefficients(newCtrlPoints);
    aU = coefficients[0][0];
    bU = coefficients[1][0];
    cU = coefficients[2][0];
    dU = coefficients[3][0];
    aV = coefficients[0][1];
    bV = coefficients[1][1];
    cV = coefficients[2][1];
    dV = coefficients[3][1];

    this->cubic_bezier = CubicBezier2D(newCtrlPoints);
    this->cubic_bezier.arclen_t[length] = 1.0;
    this->cubic_bezier.valid_length = length;
}

} // namespace odr
