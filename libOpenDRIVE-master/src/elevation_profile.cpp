#include "road_profile.h"
#include <cassert>

namespace LM
{
    double CubicSplineGenerator::Precision = 1e-3;

    void CubicSplineGenerator::OverwriteSection(odr::CubicSpline& target, double length, 
        double start, double end, double value) 
    {
        assert(0 <= start);
        assert(start <= end);
        assert(end <= length);
        auto& table = target.s0_to_poly;
        if (table.empty())
            table.emplace(0, odr::Poly3(0, 0, 0, 0, 0));

        // Snap keys
        const auto existingKeys = odr::get_map_keys(table);
        if (start > length - Precision)
        {
            start = length;
        }
        odr::snapToExistingKey(existingKeys, start, Precision);
        if (end > length - Precision) 
        {
            end = length;
        }
        odr::snapToExistingKey(existingKeys, end, Precision);

        if (std::abs(target.get(start) - value) < Precision &&
            std::abs(target.get(end) - value) < Precision) 
        {
            // already satisfied
            return;
        }

        // erase keys where start < key < end
        auto keyBegin = table.lower_bound(start);
        auto keyBefore = keyBegin;
        if (start != 0)
            keyBefore--;
        auto keyAfter = table.upper_bound(end);

        double nextS = keyAfter == table.end() ? length : keyAfter->first;
        double nextY = target.get(nextS);
        double prevS = keyBefore->first;
        double prevY = target.get(prevS);

        // erase all keys within range [start, end]
        table.erase(keyBegin, keyAfter);

        // put a straight line between start and end
        if (start != length)
        {
            table[start] = odr::Poly3(start, value, 0, 0, 0);
        }
        if (prevS != start) 
        {
            table[prevS] = FitPoly3(prevS, prevY, start, value);
        }
        if (end != nextS) 
        {
            table[end] = FitPoly3(end, value, nextS, nextY);
        }
    }

    odr::Poly3 CubicSplineGenerator::FitPoly3(double s0, double y0, double s1, double y1) 
    { 
        double l = s1 - s0;
        double a = y0;
        double c = -3 * (y0 - y1) / pow(l, 2);
        double d = 2 * (y0 - y1) / pow(l, 3);
        auto rtn = odr::Poly3(s0, a, 0, c, d);
        return rtn;
    }

    odr::CubicSpline CubicSplineGenerator::FromControlPoints(std::map<double, double> keyPoints) 
    { 
        assert(keyPoints.size() >= 2);

        std::map<double, double> keySlopes;
        for (auto x_y = keyPoints.begin(); x_y != keyPoints.end(); ++x_y) 
        {
            if (x_y == keyPoints.begin()) 
            {
                keySlopes.emplace(x_y->first, 0);
                continue;
            }
            auto next = x_y;
            next++;
            if (next == keyPoints.end()) 
            {
                keySlopes.emplace(x_y->first, 0);
                continue;
            }
            auto prev = x_y;
            prev--;
            auto prev_slope = (prev->second - x_y->second) / (prev->first - x_y->first);
            auto next_slope = (next->second - x_y->second) / (next->first - x_y->first);
            double keySlope = 0;
            if (prev_slope > 0 == next_slope > 0) 
            {
                keySlope = std::sqrt(prev_slope * next_slope);
                if (prev_slope < 0) 
                {
                    keySlope = -keySlope;
                }
            }
            keySlopes.emplace(x_y->first, keySlope);
        }

        odr::CubicSpline rtn;
        for (auto x_y = keyPoints.begin();; ++x_y) 
        {
            auto next = x_y;
            next++;
            if (next == keyPoints.end()) 
            {
                break;
            }
            auto x0 = x_y->first, x1 = next->first;
            auto y0 = x_y->second, y1 = next->second;
            auto y_0 = keySlopes.at(x0), y_1 = keySlopes.at(x1);
            rtn.s0_to_poly.emplace(x0, FitPoly3(x0, y0, y_0, x1, y1, y_1));
        }

        return rtn;
    }

    odr::Poly3 CubicSplineGenerator::FitPoly3(double s0, double y0, double y_0, double s1, double y1, double y_1)
    { 
        double l = s1 - s0;
        y_0 *= l;
        y_1 *= l;
        double a = y0;
        double b = y_0;
        double c = 3 * y1 - y_1 - 3 * y0 - 2 * y_0;
        double d = -2 * y1 + y_1 + 2 * y0 + y_0; 
        return odr::Poly3(s0, a, b / l, c / pow(l, 2), d / pow(l, 3));
    }
 }
