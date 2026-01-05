#include "spatial_indexer.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/intersections.h>
#include <CGAL/Polygon_2.h>

using Kernel   = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_2  = Kernel::Point_2;
using Point_3  = Kernel::Point_3;
using Segment_2 = Kernel::Segment_2;
using Polygon_2 = CGAL::Polygon_2<Kernel>;

namespace LM {
    bool Overlap2D(const Quad& q1, const Quad& q2) {
        Polygon_2 p1, p2;

        p1.push_back(Point_2(q1.pointS1T1[0], q1.pointS1T1[1]));
        p1.push_back(Point_2(q1.pointS1T2[0], q1.pointS1T2[1]));
        p1.push_back(Point_2(q1.pointS2T2[0], q1.pointS2T2[1]));
        p1.push_back(Point_2(q1.pointS2T1[0], q1.pointS2T1[1]));

        p2.push_back(Point_2(q2.pointS1T1[0], q2.pointS1T1[1]));
        p2.push_back(Point_2(q2.pointS1T2[0], q2.pointS1T2[1]));
        p2.push_back(Point_2(q2.pointS2T2[0], q2.pointS2T2[1]));
        p2.push_back(Point_2(q2.pointS2T1[0], q2.pointS2T1[1]));

        // 1. Edge-edge intersections
        for (auto ea = p1.edges_begin(); ea != p1.edges_end(); ++ea)
            for (auto eb = p2.edges_begin(); eb != p2.edges_end(); ++eb)
                if (CGAL::do_intersect(*ea, *eb))
                    return true;

        // 2. One contains a vertex of the other
        if (p1.is_simple() && p1.bounded_side(p2[0]) != CGAL::ON_UNBOUNDED_SIDE) return true;
        if (p2.is_simple() && p2.bounded_side(p1[0]) != CGAL::ON_UNBOUNDED_SIDE) return true;

        return false;
    }

    bool Overlap3D(const Quad& q1, const Quad& q2, double zThreshold) {
        std::array<Point_3, 4> line1 = {
            Point_3(q1.pointS1T1[0], q1.pointS1T1[1], q1.pointS1T1[2]),
            Point_3(q1.pointS1T2[0], q1.pointS1T2[1], q1.pointS1T2[2]),
            Point_3(q1.pointS2T2[0], q1.pointS2T2[1], q1.pointS2T2[2]),
            Point_3(q1.pointS2T1[0], q1.pointS2T1[1], q1.pointS2T1[2])
        };

        std::array<Point_3, 4> line2 = {
            Point_3(q2.pointS1T1[0], q2.pointS1T1[1], q2.pointS1T1[2]),
            Point_3(q2.pointS1T2[0], q2.pointS1T2[1], q2.pointS1T2[2]),
            Point_3(q2.pointS2T2[0], q2.pointS2T2[1], q2.pointS2T2[2]),
            Point_3(q2.pointS2T1[0], q2.pointS2T1[1], q2.pointS2T1[2])
        };

        // Case1: Edge-Edge intersection
        // Assuming p11-p12 / p21-p22 (XY projection) intersect at intersection
        auto zAtIntersection = [](const Point_3& p1, const Point_3& p2, const Point_2& p1_2, const Point_2& p2_2,
            const Point_2& intersection) {
            auto portion = std::sqrt(CGAL::squared_distance(intersection, p1_2) / CGAL::squared_distance(p1_2, p2_2));
            return p1.z() * (1 - portion) + p2.z() * portion;
        };

        for (int i1 = 0; i1 != 4; ++i1) {
            Point_3& pt11 = line1[i1];
            Point_3& pt12 = line1[(i1 + 1) % 4];
            Point_2 pt11_2(pt11.x(), pt11.y());
            Point_2 pt12_2(pt12.x(), pt12.y());

            Segment_2 s1(pt11_2, pt12_2);

            for (int i2 = 0; i2 != 4; ++i2) {
                Point_3& pt21 = line2[i2];
                Point_3& pt22 = line2[(i2 + 1) % 4];
                Point_2 pt21_2(pt21.x(), pt21.y());
                Point_2 pt22_2(pt22.x(), pt22.y());

                Segment_2 s2(pt21_2, pt22_2);

                if (const auto result = CGAL::intersection(s1, s2)) {
                    if (const Point_2* p = boost::get<Point_2>(&*result)) {
                        auto h1 = zAtIntersection(pt11, pt12, pt11_2, pt12_2, *p);
                        auto h2 = zAtIntersection(pt21, pt22, pt21_2, pt22_2, *p);
                        if (std::abs(h1 - h2) < zThreshold) {
                            return true;
                        }
                    }
                }
            }
        }

        // Case 2: vertex fall inside the other quad

        // p0-p1 and p1-p2 form two edges
        // Assuming loc (XY projection) is inside the quod
        auto projLengthToQuad = [](const Point_3& p0, const Point_3& p1, const Point_3& p2, const Point_3& loc) {
            const Point_2 v1(p1.x() - p0.x(), p1.y() - p0.y());
            const Point_2 v2(p2.x() - p0.x(), p2.y() - p0.y());
            const Point_2 v3(loc.x() - p0.x(), loc.y() - p0.y());
            // Solve for s * v1 + t * v2 = v3
            auto det = v1.x() * v2.y() - v1.y() * v2.x();
            auto s = (v3.x() * v2.y() - v3.y() * v2.x()) / det;
            auto t = (v1.x() * v3.y() - v1.y() * v3.x()) / det;
            auto zOnQuad = p0.z() + (p1.z() - p0.z()) * s + (p2.z() - p0.z()) * t;
            return std::abs(loc.z() - zOnQuad);
        };

        Polygon_2 poly1, poly2;

        poly1.push_back(Point_2(q1.pointS1T1[0], q1.pointS1T1[1]));
        poly1.push_back(Point_2(q1.pointS1T2[0], q1.pointS1T2[1]));
        poly1.push_back(Point_2(q1.pointS2T2[0], q1.pointS2T2[1]));
        poly1.push_back(Point_2(q1.pointS2T1[0], q1.pointS2T1[1]));

        poly2.push_back(Point_2(q2.pointS1T1[0], q2.pointS1T1[1]));
        poly2.push_back(Point_2(q2.pointS1T2[0], q2.pointS1T2[1]));
        poly2.push_back(Point_2(q2.pointS2T2[0], q2.pointS2T2[1]));
        poly2.push_back(Point_2(q2.pointS2T1[0], q2.pointS2T1[1]));

        for (const auto& v1: line1) {
            if (poly2.is_simple() && poly2.bounded_side(Point_2(v1.x(), v1.y())) != CGAL::ON_UNBOUNDED_SIDE) {
                if (projLengthToQuad(line2[0], line2[1], line2[3], v1) < zThreshold) {
                    return true;
                }
            }
        }
        for (const auto& v2: line2) {
            if (poly1.is_simple() && poly1.bounded_side(Point_2(v2.x(), v2.y())) != CGAL::ON_UNBOUNDED_SIDE) {
                if (projLengthToQuad(line1[0], line1[1], line1[3], v2) < zThreshold) {
                    return true;
                }
            }
        }
        return false;
    }
}