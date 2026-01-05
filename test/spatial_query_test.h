#ifndef LANEMAKER_SPATIAL_QUERY_TEST_H
#define LANEMAKER_SPATIAL_QUERY_TEST_H

#include <gtest/gtest.h>

#include <CGAL/Bbox_2.h>
#include <CGAL/box_intersection_d.h>
typedef CGAL::Box_intersection_d::Box_d<double,2> Box;
typedef CGAL::Bbox_2                              Bbox;

class AABB
{
public:
    typedef double NT;
    typedef unsigned long ID;

    AABB(ID id, NT xmin, NT ymin, NT xmax, NT ymax): _id(id) {
        _min[0] = xmin;
        _min[1] = ymin;
        _max[0] = xmax;
        _max[1] = ymax;
    }

    static int dimension () {return 2;}
    ID id() const {return _id;}

    double min_coord(int& dim) const { return _min[dim]; }
    double max_coord(int& dim) const { return _max[dim]; }

private:
    ID _id;
    double _min[2];
    double _max[2];
};

namespace LTest
{
    TEST(SpatialQuery, SampleUsage)
    {
        std::vector groupA = {
            AABB(0, 0, 0, 2, 2),
            AABB(0, 2, 2, 3, 3)
        };

        std::vector groupB = {
            AABB(1, 0, -1, 1, 1),
            AABB(1, -10, -10, 40, 42)
        };

        struct Callback {
            void operator()(const AABB& a, const AABB& b) {
                std::cout << "Intersect found between " << a.id() << " and " << b.id() << std::endl;
            }
        };

        CGAL::box_intersection_d(
            groupA.begin(), groupA.end(),
            groupB.begin(), groupB.end(),
            Callback());

    }
}

#endif //LANEMAKER_SPATIAL_QUERY_TEST_H