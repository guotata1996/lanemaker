#include "Geometries/CubicSpline.h"
#include "CubicBezier.hpp"
#include "Math.hpp"
#include "Utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <iostream>

namespace odr
{
Poly3::Poly3(double s0, double a, double b, double c, double d) : raw_a(a), raw_b(b), raw_c(c), raw_d(d)
{
    /* ds = s - s0 => resolve to polynomial form */
    /* make poly3s work on absolute s position => makes CubicSpline::add work */
    this->a = a - b * s0 + c * s0 * s0 - d * s0 * s0 * s0;
    this->b = b - 2 * c * s0 + 3 * d * s0 * s0;
    this->c = c - 3 * d * s0;
    this->d = d;
}

void Poly3::ComputeRelative(double s0) 
{ 
    raw_d = d;
    raw_c = c + 3 * raw_d * s0;
    raw_b = b + 2 * raw_c * s0 - 3 * raw_d * s0 * s0;
    raw_a = a + raw_b * s0 - raw_c * s0 * s0 + raw_d * s0 * s0 * s0;
}

double Poly3::get(double s) const { return a + b * s + c * s * s + d * s * s * s; }

double Poly3::get_grad(double s) const { return b + 2 * c * s + 3 * d * s * s; }

double Poly3::get_max(double s_start, double s_end) const
{
    const auto endPointMax = std::max(this->get(s_start), this->get(s_end));
    if (this->d != 0)
    {
        const double s_extr = (std::sqrt(c * c - 3 * b * d) - c) / (3 * d);
        const double max_val1 = this->get(std::min(std::max(s_extr, s_start), s_end));
        const double max_val2 = this->get(std::min(std::max(-s_extr, s_start), s_end));
        return std::max(endPointMax, std::max(max_val1, max_val2));
    }
    else if (this->c != 0)
    {
        const double s_extr = (-b) / (2 * c);
        return std::max(endPointMax, this->get(std::min(std::max(s_extr, s_start), s_end)));
    }

    return endPointMax;
}

std::set<double> Poly3::approximate_linear(double eps, double s_start, double s_end) const
{
    if (s_start == s_end)
        return {};

    if (d == 0 && c == 0)
        return {s_start, s_end};

    std::vector<double> s_vals;
    if (d == 0 && c != 0)
    {
        double s = s_start;
        while (s < s_end)
        {
            s_vals.push_back(s);
            s = (c != 0) ? s + std::sqrt(std::abs(eps / c)) : s + eps;
        }
    }
    else
    {
        /* transform to parametric form */
        const double& s_0 = s_start;
        const double& s_1 = s_end;
        const double  d_p = -d * s_0 * s_0 * s_0 + d * s_1 * s_1 * s_1 - 3 * d * s_0 * s_1 * s_1 + 3 * d * s_0 * s_0 * s_1;
        const double  c_p =
            3 * d * s_0 * s_0 * s_0 + 3 * d * s_0 * s_1 * s_1 - 6 * d * s_0 * s_0 * s_1 + c * s_0 * s_0 + c * s_1 * s_1 - 2 * c * s_0 * s_1;
        const double b_p = -3 * d * s_0 * s_0 * s_0 + 3 * d * s_0 * s_0 * s_1 - 2 * c * s_0 * s_0 + 2 * c * s_0 * s_1 - b * s_0 + b * s_1;
        const double a_p = d * s_0 * s_0 * s_0 + c * s_0 * s_0 + b * s_0 + a;

        const std::array<Vec1D, 4> coefficients = {{{a_p}, {b_p}, {c_p}, {d_p}}};
        const std::set<double>     p_vals = CubicBezier1D(CubicBezier1D::get_control_points(coefficients)).approximate_linear(eps);

        s_vals.push_back(s_start);
        for (const double& p : p_vals)
            s_vals.push_back(p * (s_end - s_start) + s_start);
    }

    if ((s_end - s_vals.back()) < 1e-9 && (s_vals.size() != 1))
        s_vals.back() = s_end;
    else
        s_vals.push_back(s_end);

    std::set<double> s_vals_set(s_vals.begin(), s_vals.end());
    return s_vals_set;
}

void Poly3::negate()
{
    a = -a;
    b = -b;
    c = -c;
    d = -d;
}

bool Poly3::isnan() const { return (std::isnan(this->a) || std::isnan(this->b) || std::isnan(this->c) || std::isnan(this->d)); }

bool CubicSpline::empty() const { return this->s0_to_poly.empty(); }

std::size_t CubicSpline::size() const { return this->s0_to_poly.size(); }

CubicSpline::CubicSpline(double height) 
{ 
    s0_to_poly.emplace(0, odr::Poly3(0, 0, 0, 0, 0)); 
}

double CubicSpline::get(double s, double default_val, bool extend_start) const
{
    const Poly3& poly = this->get_poly(s, extend_start);
    if (poly.isnan())
        return default_val;
    return poly.get(s);
}

double CubicSpline::get_grad(double s, double default_val, bool extend_start) const
{
    const Poly3& poly = this->get_poly(s, extend_start);
    if (poly.isnan())
        return default_val;
    return poly.get_grad(s);
}

CubicSpline CubicSpline::negate() const
{
    CubicSpline negated = *this;
    for (auto& s0_poly : negated.s0_to_poly)
        s0_poly.second.negate();
    return negated;
}

CubicSpline CubicSpline::add(const CubicSpline& other) const
{
    if (other.s0_to_poly.empty())
        return *this;
    if (this->s0_to_poly.empty())
        return other;

    std::set<double> s0_vals = get_map_keys(this->s0_to_poly);
    std::set<double> other_s0s = get_map_keys(other.s0_to_poly);
    s0_vals.insert(other_s0s.begin(), other_s0s.end());

    CubicSpline retval;
    for (const double& s0 : s0_vals)
    {
        const Poly3& this_poly = this->get_poly(s0, false);
        const Poly3& other_poly = other.get_poly(s0, false);

        if (this_poly.isnan() || other_poly.isnan()) // can't be both NAN
        {
            retval.s0_to_poly[s0] = this_poly.isnan() ? other_poly : this_poly;
            continue;
        }

        Poly3 res;
        res.a = this_poly.a + other_poly.a;
        res.b = this_poly.b + other_poly.b;
        res.c = this_poly.c + other_poly.c;
        res.d = this_poly.d + other_poly.d;
        retval.s0_to_poly[s0] = res;
    }
    return retval;
}

Poly3 CubicSpline::get_poly(double s, bool extend_start) const
{
    if (this->s0_to_poly.empty())
        return Poly3(NAN, NAN, NAN, NAN, NAN);

    if ((extend_start == false) && (s < this->s0_to_poly.begin()->first))
        return Poly3(NAN, NAN, NAN, NAN, NAN);

    // will return first poly if s < s_start and last poly for s > s_end
    auto target_poly_iter = this->s0_to_poly.upper_bound(s);
    if (target_poly_iter != this->s0_to_poly.begin())
        target_poly_iter--;
    return target_poly_iter->second;
}

double CubicSpline::get_max(double s_start, double s_end) const
{
    if ((s_start == s_end) || this->s0_to_poly.empty())
        return 0;

    auto s_end_poly_iter = this->s0_to_poly.lower_bound(s_end);
    auto s_start_poly_iter = this->s0_to_poly.upper_bound(s_start);
    if (s_start_poly_iter != this->s0_to_poly.begin())
        s_start_poly_iter--;

    std::vector<double> max_poly_vals;
    for (auto s_poly_iter = s_start_poly_iter; s_poly_iter != s_end_poly_iter; s_poly_iter++)
    {
        const double s_start_poly = std::max(s_poly_iter->first, s_start);
        const double s_end_poly = (std::next(s_poly_iter) == s_end_poly_iter) ? s_end : std::min(std::next(s_poly_iter)->first, s_end);
        max_poly_vals.push_back(s_poly_iter->second.get_max(s_start_poly, s_end_poly));
    }

    const auto   max_iter = std::max_element(max_poly_vals.begin(), max_poly_vals.end());
    const double max_val = (max_iter == max_poly_vals.end()) ? 0 : *max_iter;
    return max_val;
}

double CubicSpline::get_min(double s_start, double s_end) const 
{ 
    return -negate().get_max(s_start, s_end); 
}

void CubicSpline::reverse(double length) 
{
    decltype(s0_to_poly) reversed;
    for (auto it = s0_to_poly.begin(); it != s0_to_poly.end(); ++it) 
    {
        auto   endIt = it;
        endIt++;
        double revS = endIt == s0_to_poly.end() ? 0 : length - endIt->first;
        double segLen = endIt == s0_to_poly.end() ? length - it->first : endIt->first - it->first;

        auto&  poly = it->second;
        // old_x(local) <- new_x - x
        double revRawA = poly.raw_a + poly.raw_b * segLen + poly.raw_c * pow(segLen, 2) + poly.raw_d * pow(segLen, 3);
        double revRawB = -poly.raw_b - 2 * poly.raw_c * segLen - 3 * poly.raw_d * pow(segLen, 2);
        double revRawC = poly.raw_c + 3 * poly.raw_d * segLen;
        double revRawD = -poly.raw_d;
        odr::Poly3 revSeg(revS, revRawA, revRawB, revRawC, revRawD);
        reversed.emplace(revS, std::move(revSeg));
    }

    s0_to_poly = reversed;
}

CubicSpline CubicSpline::split(double s) 
{ 
    CubicSpline rtn;
    if (s0_to_poly.empty())
        return rtn;
    auto secondaryBegin = s0_to_poly.upper_bound(s);
    secondaryBegin--;
    if (secondaryBegin->first != s) 
    {
        Poly3 splitAt = secondaryBegin->second;
        double deltaS0 = s - secondaryBegin->first;
        // old_x(local) = new_x + deltaS0
        double shiftRawA = splitAt.raw_a + splitAt.raw_b * deltaS0 + splitAt.raw_c * pow(deltaS0, 2) + splitAt.raw_d * pow(deltaS0, 3);
        double shiftRawB = splitAt.raw_b + 2 * splitAt.raw_c * deltaS0 + 3 * splitAt.raw_d * pow(deltaS0, 2);
        double shiftRawC = splitAt.raw_c + 3 * splitAt.raw_d * deltaS0;
        double shiftRawD = splitAt.raw_d;
        rtn.s0_to_poly.emplace(0, Poly3(0, shiftRawA, shiftRawB, shiftRawC, shiftRawD));
        secondaryBegin++;
    }
    for (auto it = secondaryBegin; it != s0_to_poly.end(); ++it) 
    {
        double sInSecond = it->first - s;
        Poly3& poly = it->second;
        rtn.s0_to_poly.emplace(sInSecond, Poly3(sInSecond, poly.raw_a, poly.raw_b, poly.raw_c, poly.raw_d));
    }
    s0_to_poly.erase(secondaryBegin, s0_to_poly.end());
    return rtn;
}

void CubicSpline::join(double length, const CubicSpline& second)
{
    for (const auto& s_poly : second.s0_to_poly) 
    {
        Poly3 poly = s_poly.second;
        s0_to_poly.emplace(length + s_poly.first, 
            Poly3(length + s_poly.first, poly.raw_a, poly.raw_b, poly.raw_c, poly.raw_d));
    }
}

std::set<double> CubicSpline::approximate_linear(double eps, double s_start, double s_end) const
{
    if ((s_start == s_end) || this->s0_to_poly.empty())
        return {};

    auto s_end_poly_iter = this->s0_to_poly.lower_bound(s_end);
    auto s_start_poly_iter = this->s0_to_poly.upper_bound(s_start);
    if (s_start_poly_iter != this->s0_to_poly.begin())
        s_start_poly_iter--;

    std::set<double> s_vals;
    for (auto s_poly_iter = s_start_poly_iter; s_poly_iter != s_end_poly_iter; s_poly_iter++)
    {
        const double s_start_poly = std::max(s_poly_iter->first, s_start);
        const double s_end_poly = (std::next(s_poly_iter) == s_end_poly_iter) ? s_end : std::min(std::next(s_poly_iter)->first, s_end);

        std::set<double> s_vals_poly = s_poly_iter->second.approximate_linear(eps, s_start_poly, s_end_poly);
        if (s_vals_poly.size() < 2)
        {
            std::string err_msg = std::string("expected at least two sample points, got ") + std::to_string(s_vals_poly.size()) +
                                  std::string(" for [") + std::to_string(s_start_poly) + ' ' + std::to_string(s_end_poly) + ']';
            throw std::runtime_error(err_msg);
        }

        s_vals.insert(s_vals_poly.begin(), s_vals_poly.end());
    }

    return s_vals;
}

std::string CubicSpline::ToString() const
{
    std::stringstream ss;
    ss << "\n====Elevation Profile====\n";
    for (const auto& s_poly : s0_to_poly)
    {
        ss << s_poly.first << " :a= " << s_poly.second.raw_a << " b= " << s_poly.second.raw_b << " c= " << s_poly.second.raw_c
           << " d= " << s_poly.second.raw_d << std::endl;
    }
    return ss.str();
}

} // namespace odr
