#include "polyline.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_non_caching_segment_traits_2.h>
#include <CGAL/Arrangement_with_history_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Arr_non_caching_segment_traits_2<Kernel>  Traits_2;
typedef Traits_2::Point_2                               Point;
typedef Traits_2::X_monotone_curve_2                    Segment;
typedef CGAL::Arrangement_with_history_2<Traits_2>      Arrangement;
typedef Arrangement::Halfedge_around_vertex_circulator  Halfedge_circulator;
typedef Kernel::FT                                      FT;

namespace RoadRunner
{
    std::string Serialize(Point p)
    {
        std::string rtn = p.exact().x().get_str();
        rtn += p.exact().y().get_str();
        return rtn;
    }

    std::string Serialize(Segment s)
    {
        std::string rtn = Serialize(s.start());
        rtn += Serialize(s.end());
        return rtn;
    }

    std::map<std::string, double> addToArrangement(Arrangement& arrangement, const odr::Road& road, const int side,
        const odr::RoadLink::ContactPoint searchBegin, const double SearchLimit)
    {
        const double Resolution = 1.0;

        std::map<std::string, double> pointToS;
        double searchBeginS = searchBegin == odr::RoadLink::ContactPoint_End ? road.length : 0;
        double searchStep = searchBegin == odr::RoadLink::ContactPoint_End ? -Resolution : Resolution;

        for (double s1 = searchBeginS, s2 = searchBeginS + searchStep;
            0 <= s1 && s1 <= road.length && 0 <= s2 && s2 <= road.length &&
            std::abs(s1 - searchBeginS) < SearchLimit &&
            std::abs(s2 - searchBeginS) < SearchLimit
            ; s1 += searchStep, s2 += searchStep)
        {
            auto p1 = road.get_boundary_xy(side, s1);
            auto p2 = road.get_boundary_xy(side, s2);
            Point point1(p1[0], p1[1]);
            Point point2(p2[0], p2[1]);
            Segment segment(point1, point2);
            pointToS.emplace(Serialize(point1), s1);
            pointToS.emplace(Serialize(point2), s2);
            CGAL::insert(arrangement, segment);
        }

        return pointToS;
    }

    bool borderIntersect(const odr::Road& roadA, const int sideA,
        const odr::Road& roadB, const int sideB, double& outSA, double& outSB,
        const odr::RoadLink::ContactPoint searchBeginA,
        const odr::RoadLink::ContactPoint searchBeginB,
        const double SearchLimit)
    {
        Arrangement arrangement;
        std::map<std::string, double> pointAToS = addToArrangement(arrangement, roadA, sideA, searchBeginA, SearchLimit);
        std::map<std::string, double> pointBToS = addToArrangement(arrangement, roadB, sideB, searchBeginB, SearchLimit);

        outSA = searchBeginA == odr::RoadLink::ContactPoint_End ? -1e9 : 1e9;
        outSB = searchBeginB == odr::RoadLink::ContactPoint_End ? -1e9 : 1e9;
        bool iFound = false;
        // special case A: sample vertice overlap
        for (auto key : pointAToS)
        {
            if (pointBToS.find(key.first) != pointBToS.end())
            {
                iFound = true;
                auto candSA = key.second;
                auto candSB = pointBToS.at(key.first);
                outSA = searchBeginA == odr::RoadLink::ContactPoint_End ? std::max(outSA, candSA) : std::min(outSA, candSA);
                outSB = searchBeginB == odr::RoadLink::ContactPoint_End ? std::max(outSB, candSB) : std::min(outSB, candSB);
            }
            if (iFound)
            {
                return true;
            }
        }

        for (auto vIt = arrangement.vertices_begin(); vIt != arrangement.vertices_end(); ++vIt)
        {
            if (vIt->degree() <= 2)
            {
                continue;
            }
            std::vector<double> sMarksA, sMarksB;
            std::vector<double> heLengthA, heLengthB;

            auto vSer = Serialize(vIt->point());
            if (pointAToS.find(vSer) != pointAToS.end())
            {
                // special case B
                sMarksA.push_back(pointAToS.at(vSer));
                heLengthA.push_back(0);
            }
            if (pointBToS.find(vSer) != pointBToS.end())
            {
                // special case B
                sMarksB.push_back(pointBToS.at(vSer));
                heLengthB.push_back(0);
            }

            // Normal case
            Halfedge_circulator start = vIt->incident_halfedges();
            Halfedge_circulator circulator = start;
            do
            {
                FT squared_length = CGAL::squared_distance(circulator->source()->point(), circulator->target()->point());
                double heLength = std::sqrt(CGAL::to_double(squared_length));

                if (vIt != circulator->source())
                {
                    auto startSer = Serialize(circulator->source()->point());
                    if (pointAToS.find(startSer) != pointAToS.end())
                    {
                        sMarksA.push_back(pointAToS.at(startSer));
                        heLengthA.push_back(heLength);
                    }
                    if (pointBToS.find(startSer) != pointBToS.end())
                    {
                        sMarksB.push_back(pointBToS.at(startSer));
                        heLengthB.push_back(heLength);
                    }
                }

                if (vIt != circulator->target())
                {
                    auto endSer = Serialize(circulator->target()->point());
                    if (pointAToS.find(endSer) != pointAToS.end())
                    {
                        sMarksA.push_back(pointAToS.at(endSer));
                        heLengthA.push_back(heLength);
                    }
                    if (pointBToS.find(endSer) != pointBToS.end())
                    {
                        sMarksB.push_back(pointBToS.at(endSer));
                        heLengthB.push_back(heLength);
                    }
                }
                ++circulator;
            } while (circulator != start);
            if (sMarksA.size() != 2)
            {
                spdlog::trace("Road {} has self-intersection: {}", roadA.id, sMarksA.size());
                continue;
            }
            if (sMarksB.size() != 2)
            {
                spdlog::trace("Road {} has self-intersection: {}", roadB.id, sMarksB.size());
                continue;
            }

            iFound = true;
            auto candSA = (sMarksA[0] * heLengthA[1] + sMarksA[1] * heLengthA[0]) / (heLengthA[0] + heLengthA[1]);
            auto candSB = (sMarksB[0] * heLengthB[1] + sMarksB[1] * heLengthB[0]) / (heLengthB[0] + heLengthB[1]);
            outSA = searchBeginA == odr::RoadLink::ContactPoint_End ? std::max(outSA, candSA) : std::min(outSA, candSA);
            outSB = searchBeginB == odr::RoadLink::ContactPoint_End ? std::max(outSB, candSB) : std::min(outSB, candSB);
        }
        return iFound;
    }
}
