#include <CGAL/intersections.h>
#include <CGAL/Cartesian.h>

#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
#include "CubicBezier.hpp"

#include "spdlog/spdlog.h"

using namespace CGAL;

typedef Cartesian<double>  Kernel;

// l1 --> conn --> l2
boost::optional<odr::ParamPoly3> ConnectLines(const odr::Line& l1, const odr::Line& l2)
{
    auto p1_end = l1.get_xy(l1.length);
    Point_2 <Kernel> p1(p1_end[0], p1_end[1]), p2(l2.x0, l2.y0);
    Direction_2< Kernel > dir1(std::cos(l1.hdg0), std::sin(l1.hdg0));
    Direction_2< Kernel> dir2(std::cos(l2.hdg0), std::sin(l2.hdg0));
    Ray_2< Kernel > ray1(p1, dir1);
    Ray_2< Kernel > ray2(p2, -dir2);

    Object result = intersection(ray1, ray2);
    boost::optional<odr::ParamPoly3> rtn;
    
    if (const CGAL::Point_2<Kernel>* pm = object_cast<Point_2<Kernel>>(&result)) 
    {
        // Converging case
        odr::Vec2D odr_p0({ p1.x(), p1.y()});
        odr::Vec2D odr_p1({ pm->x(), pm->y() });
        odr::Vec2D odr_p2({ p2.x(), p2.y() });
        odr::CubicBezier2D curve({ odr_p0, odr_p1, odr_p1, odr_p2 });
        spdlog::info("Result 2-Bezier length: {}", curve.get_length());

        auto coefficients = CubicBezier2D::get_coefficients(curve.control_points);
        rtn.emplace(0, p1_end[0], p1_end[1], l1.hdg0, curve.get_length(),
            coefficients[0][0], coefficients[1][0], coefficients[2][0], coefficients[3][0],
            coefficients[1][0], coefficients[1][1], coefficients[2][1], coefficients[3][1],
            true);
        return rtn;
    }
    
    ray1 = Ray_2< Kernel >(p1, -dir1);
    ray2 = Ray_2< Kernel >(p2, dir2);
    result = intersection(ray1, ray2);
    if (const CGAL::Point_2<Kernel>* pm = object_cast<Point_2<Kernel>>(&result))
    {
        // diverging case
        double dist = std::sqrt(squared_distance(p1, p2));
        Point_2 <Kernel> c1 = p1;
        c1 += dir1.to_vector() * dist;
        odr::Vec2D odr_p0({ p1.x(), p1.y() });
        odr::Vec2D odr_p1({ c1.x(), c1.y() });
        Point_2 <Kernel> c2 = p2;
        c2 -= dir2.to_vector() * dist;
        odr::Vec2D odr_p2({ c2.x(), c2.y() });
        odr::Vec2D odr_p3({ p2.x(), p2.y() });
        odr::CubicBezier2D curve({ odr_p0, odr_p1, odr_p2, odr_p3 });
        spdlog::info("Result 3-Bezier length: {}", curve.get_length());

        auto coefficients = CubicBezier2D::get_coefficients(curve.control_points);
        rtn.emplace(0, p1_end[0], p1_end[1], l1.hdg0, curve.get_length(),
            coefficients[0][0], coefficients[1][0], coefficients[2][0], coefficients[3][0],
            coefficients[1][0], coefficients[1][1], coefficients[2][1], coefficients[3][1],
            true);
        return rtn;
    }

    return rtn;

}