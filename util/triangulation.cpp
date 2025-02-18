#include "triangulation.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Polygon_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Point_2<K> Point;
typedef CGAL::Constrained_Delaunay_triangulation_2<K> CDT;
typedef CGAL::Polygon_2<K> Polygon_2;

#include <spdlog/spdlog.h>
namespace LM
{
    bool RayLiesInside(const Polygon_2& poly, const bool ccw, int from, int to)
    {
        int prev = from == 0 ? poly.size() - 1 : from - 1;
        int next = from == poly.size() - 1 ? 0 : from + 1;
        if (to == prev || to == next)
        {
            return true;
        }
        auto a = poly[from] - poly[prev];
        auto b = poly[next] - poly[from];
        odr::Vec2D aa{ a.x(), a.y() };
        odr::Vec2D bb{ b.x(), b.y() };
        auto aToB = odr::angle(aa, bb);

        auto c = poly[to] - poly[from];
        odr::Vec2D cc{c.x(), c.y() };
        auto aToC = odr::angle(aa, cc);
        return ccw == (aToB < aToC);
    }

    std::vector<std::tuple<int, int, int>> Triangulate_2_5d(const odr::Line3D& boundary)
    {
        std::vector<std::tuple<int, int, int>> rtn;

        std::map<Point, int> pointIndex;

        Polygon_2 polygon;

        for (int i = 0; i != boundary.size(); ++i)
        {
            Point p2d(boundary[i][0], boundary[i][1]);
            pointIndex.emplace(p2d, i);
            polygon.push_back(p2d);
        }
        bool ccw;
        try
        {
            ccw = polygon.is_counterclockwise_oriented();
        }
        catch (CGAL::Precondition_exception)
        {
            spdlog::warn("Invalid geometry to triangulate!");
            return rtn;
        }

        CDT T;
        for (size_t i = 0; i < polygon.size(); ++i) {
            T.insert_constraint(polygon[i], polygon[(i + 1) % polygon.size()]);
        }

        for (auto face = T.finite_faces_begin(); face != T.finite_faces_end(); ++face)
        {   
            auto p1 = face->vertex(0)->point();
            auto p2 = face->vertex(1)->point();
            auto p3 = face->vertex(2)->point();
            auto v1 = pointIndex.at(p1);
            auto v2 = pointIndex.at(p2);
            auto v3 = pointIndex.at(p3);

            if (RayLiesInside(polygon, ccw, v1, v2) &&
                RayLiesInside(polygon, ccw, v2, v3) &&
                RayLiesInside(polygon, ccw, v3, v1))
            {
                rtn.push_back(std::make_tuple(v1, v2, v3));
            }
        }

        return rtn;
    }
}

