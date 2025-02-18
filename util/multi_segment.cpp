#include "multi_segment.h"
#include <cassert>

namespace LM
{
    MultiSegment::MultiSegment(double aIgnoreGap):
        IgnoreGap(aIgnoreGap)
    {

    }

    void MultiSegment::Insert(double a, double b)
    {
        if (a > b) std::swap(a, b);
        if (beginToDuration.find(a) != beginToDuration.end())
        {
            beginToDuration[a] = std::max(beginToDuration[a], b - a);
        }
        else
        {
            beginToDuration.emplace(a, b - a);
        }
    }

    std::vector<std::pair<double, double>> MultiSegment::Merge()
    {
        std::vector<std::pair<double, double>> rtn;
        if (beginToDuration.empty())
        {
            return rtn;
        }

        auto it = beginToDuration.begin();
        double currSegBegin = it->first;
        double currSegEnd = it->first + it->second;
        it++;

        while (it != beginToDuration.end())
        {
            if (it->first < currSegEnd + IgnoreGap)
            {
                currSegEnd = std::max(currSegEnd, it->first + it->second);
            }
            else
            {
                rtn.push_back(std::make_pair(currSegBegin, currSegEnd));

                currSegBegin = it->first;
                currSegEnd = it->first + it->second;
            }
            it++;
        }

        rtn.push_back(std::make_pair(currSegBegin, currSegEnd));

        beginToDuration.clear();
        for (const auto& merged : rtn)
        {
            beginToDuration.emplace(merged.first, merged.second - merged.first);
        }
        return rtn;
    }

    bool SegmentsIntersect(double a1, double a2, double b1, double b2)
    {
        if (a1 > a2) std::swap(a1, a2);
        if (b1 > b2) std::swap(b1, b2);
        return a1 <= b2 && b1 <= a2;
    }
}