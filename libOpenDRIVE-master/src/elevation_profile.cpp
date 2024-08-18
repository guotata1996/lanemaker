#include "road_profile.h"
#include <iostream>
#include <cassert>

namespace RoadRunner
{
    double CubicSplineGenerator::MaxTransitionLength = 60;

    void CubicSplineGenerator::OverwriteSection(odr::CubicSpline& target, double length, 
        double start, double end, double value) 
    {
        assert(0 <= start);
        assert(start <= end);
        assert(end <= length);
        auto& table = target.s0_to_poly;
        if (table.empty())
            table.emplace(0, odr::Poly3(0, 0, 0, 0, 0));

        if (std::abs(target.get(start) - value) < 1e-3 &&
            std::abs(target.get(end) - value) < 1e-3) 
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

        double nextS = keyAfter == table.end() ? std::min(length, end + MaxTransitionLength) : 
            keyAfter->first;
        double nextY = target.get(nextS);
        double prevS = std::max(keyBefore->first, start - MaxTransitionLength);
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
            if (nextS == end + MaxTransitionLength && nextS < length && table.find(nextS) == table.end())
            {
                // put straight line till end of spline
                table.emplace(nextS, odr::Poly3(nextS, nextY, 0, 0, 0));
            }
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
}
