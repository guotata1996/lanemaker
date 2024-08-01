#include "Geometries/Spiral.h"
#include "Geometries/RoadGeometry.h"
#include "Geometries/Spiral/odrSpiral.h"
#include "Math.hpp"

#include <cmath>

#include <iostream>

namespace odr
{

Spiral::Spiral(double s0, double x0, double y0, double hdg0, double length, double curv_start, double curv_end) :
    RoadGeometry(s0, x0, y0, hdg0, length, GeometryType_Spiral), curv_start(curv_start), curv_end(curv_end)
{
    this->c_dot = (curv_end - curv_start) / length;
    this->s_start = curv_start / c_dot;
    this->s_end = curv_end / c_dot;
    s0_spiral = curv_start / c_dot;
    odrSpiral(s0_spiral, c_dot, &x0_spiral, &y0_spiral, &a0_spiral);
}

std::unique_ptr<RoadGeometry> Spiral::clone() const { return std::make_unique<Spiral>(*this); }

Vec2D Spiral::get_xy(double s) const
{
    double xs_spiral, ys_spiral, as_spiral;
    odrSpiral(s - s0 + s0_spiral, c_dot, &xs_spiral, &ys_spiral, &as_spiral);

    const double hdg = hdg0 - a0_spiral;
    const double xt = (std::cos(hdg) * (xs_spiral - x0_spiral)) - (std::sin(hdg) * (ys_spiral - y0_spiral)) + x0;
    const double yt = (std::sin(hdg) * (xs_spiral - x0_spiral)) + (std::cos(hdg) * (ys_spiral - y0_spiral)) + y0;
    return Vec2D{xt, yt};
}

Vec2D Spiral::get_grad(double s) const
{
    double xs_spiral, ys_spiral, as_spiral;
    odrSpiral(s - s0 + s0_spiral, c_dot, &xs_spiral, &ys_spiral, &as_spiral);
    const double hdg = as_spiral + hdg0 - a0_spiral;
    const double dx = std::cos(hdg);
    const double dy = std::sin(hdg);
    return {{dx, dy}};
}

std::set<double> Spiral::approximate_linear(double eps) const
{
    // TODO: properly implement
    std::set<double> s_vals;
    for (double s = s0; s < (s0 + length); s += (10 * eps))
        s_vals.insert(s);
    s_vals.insert(s0 + length);

    return s_vals;
}

void Spiral::reverse() 
{ 
    RoadGeometry::reverse();
    std::swap(curv_start, curv_end);
    curv_start = -curv_start;
    curv_end = -curv_end;

    // Update underlying odrSpiral
    *this = Spiral(0, x0, y0, hdg0, length, curv_start, curv_end);
}

void Spiral::trim(double l) 
{ 
    auto frac = l / length;
    curv_end = curv_start * (1 - frac) + curv_end * frac;
    RoadGeometry::trim(l);

    // Update underlying odrSpiral
    //*this = Spiral(0, x0, y0, hdg0, length, curv_start, curv_end);
}

void Spiral::rebase(double s0) 
{ 
    auto frac = s0 / length;
    curv_start = curv_start * (1 - frac) + curv_end * frac;
    RoadGeometry::rebase(s0);

    // Update underlying odrSpiral
    *this = Spiral(0, x0, y0, hdg0, length, curv_start, curv_end);
}

double Spiral::get_closest_s_to(const Vec2D& target, double initialS) 
{
    int stepMul = 1;
    const double eps = 1e-4;

    auto pInitial = get_xy(initialS);
    auto errInitial = odr::euclDistance(pInitial, target);
    auto pPlus = get_xy(initialS + eps);
    auto errPlus = odr::euclDistance(pPlus, target);
    auto pMinus = get_xy(initialS - eps);
    auto errMinus = odr::euclDistance(pMinus, target);
    //std::cout << errInitial  << " Err +" << errPlus << " Err - " << errMinus << std::endl;
    if (errPlus > errInitial) 
    {
        if (errMinus > errInitial) {
            return initialS;
        }
        stepMul = -1;
    }

    double s0 = initialS;
    double s1 = s0 + stepMul * eps;
    double lastErr = errInitial;
    double errS1;
    while (true) 
    {
        auto pOtherEnd = get_xy(s1);
        auto errOtherEnd = odr::euclDistance(pOtherEnd, target);
        if (errOtherEnd > lastErr) 
        {
            errS1 = errOtherEnd;
            break;
        }

        lastErr = errOtherEnd;
        stepMul *= 2;
        s1 += stepMul * eps;
    }

    auto errS0 = errInitial;
    while (std::abs(s0 - s1) > eps) 
    {
        auto mid = (s0 + s1) / 2;
        auto pMid = get_xy(mid);
        auto errMid = odr::euclDistance(pMid, target);
        //std::cout << s0 << " | " << mid << "(" << errMid << ") | " << s1 << std::endl;
        if (errS0 > errS1) 
        {
            s0 = mid;
            errS0 = errMid;
        }
        else
        {
            s1 = mid;
            errS1 = errMid;
        }
    }
    return (s0 + s1) / 2;
}

} // namespace odr
