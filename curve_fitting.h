#pragma once

#include <CGAL/intersections.h>
#include <CGAL/Cartesian.h>

#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
#include "CubicBezier.hpp"

using namespace CGAL;

typedef Cartesian<double>  Kernel;

// l1 --> conn --> l2
std::shared_ptr<odr::RoadGeometry> ConnectLines(const odr::Vec2D& startPos, const odr::Vec2D& startHdg,
    const odr::Vec2D& endPos, const odr::Vec2D& endHdg)
{
    double hdg = std::atan2(startHdg[1], startHdg[0]);
    Point_2 <Kernel> p1(startPos[0], startPos[1]), p2(endPos[0], endPos[1]);
    Direction_2< Kernel > dir1(startHdg[0], startHdg[1]);
    Direction_2< Kernel> dir2(endHdg[0], endHdg[1]);
    Ray_2< Kernel > ray1(p1, dir1);
    Ray_2< Kernel > ray2(p2, -dir2);

    Object result = intersection(ray1, ray2);
    
    if (squared_distance(ray1, p2) < std::pow(1e-4, 2) && squared_distance(ray2, p1) < std::pow(1e-4, 2))
    {
        // collinear case
        return std::make_shared<odr::Line>(0, startPos[0], startPos[1], hdg, odr::euclDistance(startPos, endPos));
    }

    if (const CGAL::Point_2<Kernel>* pm = object_cast<Point_2<Kernel>>(&result)) 
    {
        // Converging case - 2nd order
        odr::Vec2D odr_p0{ 0, 0};
        odr::Vec2D odr_p1{ pm->x(), pm->y() };
        odr_p1 = odr::toLocal(odr_p1, startPos, startHdg);
        odr::Vec2D odr_p2({ p2.x(), p2.y() });
        odr_p2 = odr::toLocal(odr_p2, startPos, startHdg);

        if (odr::euclDistance(odr_p0, odr_p1) < odr::euclDistance(odr_p0, odr_p2) * 5)
        {
            // project against very far intersection point, i.e. nearly parallel hdg
            odr::CubicBezier2D curve({ odr_p0, odr_p1, odr_p1, odr_p2 });
            auto coefficients = odr::CubicBezier2D::get_coefficients(curve.control_points);
            return std::make_shared<odr::ParamPoly3>(0, startPos[0], startPos[1], hdg, curve.get_length(),
                coefficients[0][0], coefficients[1][0], coefficients[2][0], coefficients[3][0],
                coefficients[0][1], coefficients[1][1], coefficients[2][1], coefficients[3][1],
                true);
        }
    }
    
    // diverging case - 3rd order
    double dist = std::sqrt(squared_distance(p1, p2));
    Point_2 <Kernel> c1 = p1;
    c1 += dir1.to_vector() * dist / 2;
    odr::Vec2D odr_p0{ 0, 0 };
    odr::Vec2D odr_p1({ c1.x(), c1.y() });
    odr_p1 = odr::toLocal(odr_p1, startPos, startHdg);
    Point_2 <Kernel> c2 = p2;
    c2 -= dir2.to_vector() * dist / 2;
    odr::Vec2D odr_p2{ c2.x(), c2.y() };
    odr_p2 = odr::toLocal(odr_p2, startPos, startHdg);
    odr::Vec2D odr_p3{ p2.x(), p2.y() };
    odr_p3 = odr::toLocal(odr_p3, startPos, startHdg);
    odr::CubicBezier2D curve({ odr_p0, odr_p1, odr_p2, odr_p3 });

    auto coefficients = odr::CubicBezier2D::get_coefficients(curve.control_points);
    return std::make_shared<odr::ParamPoly3>(0, startPos[0], startPos[1], hdg, curve.get_length(),
        coefficients[0][0], coefficients[1][0], coefficients[2][0], coefficients[3][0],
        coefficients[0][1], coefficients[1][1], coefficients[2][1], coefficients[3][1],
        true);
}