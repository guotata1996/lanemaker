#pragma once
#include <cstddef>
#include <map>
#include <set>
#include <string>

namespace odr
{

struct Poly3
{
    Poly3() = default;
    Poly3(double s0, double a, double b, double c, double d);
    void ComputeRelative(double s0);

    double get(double s) const;
    double get_grad(double s) const;
    double get_max(double s_start, double s_end) const;
    void   negate();
    bool   isnan() const;

    std::set<double> approximate_linear(double eps, double s_start, double s_end) const;

    double a = 0;
    double b = 0;
    double c = 0;
    double d = 0;

    double raw_a = 0;
    double raw_b = 0;
    double raw_c = 0;
    double raw_d = 0;
};

struct CubicSpline
{
    CubicSpline() = default;
    CubicSpline(double height);

    double get(double s, double default_val = 0.0, bool extend_start = true) const;
    double get_grad(double s, double default_val = 0.0, bool extend_start = true) const;
    double get_max(double s_start, double s_end) const;
    double get_min(double s_start, double s_end) const;
    Poly3  get_poly(double s, bool extend_start = true) const;

    bool        empty() const;
    std::size_t size() const;
    CubicSpline negate() const;
    CubicSpline add(const CubicSpline& other) const;

    void        reverse(double length);
    CubicSpline split(double s); 
    void        join(double selfLength, const CubicSpline& second);

    std::set<double> approximate_linear(double eps, double s_start, double s_end) const;

    std::string ToString() const;

    std::map<double, Poly3> s0_to_poly;
};

} // namespace odr