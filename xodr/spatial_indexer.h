#pragma once

#include "Road.h"
#include "id_generator.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>

#include <iostream>

namespace
{
    typedef CGAL::Simple_cartesian<double> K;
    typedef K::FT FT;
    typedef K::Point_3 Point;
    typedef K::Vector_3 Vector;
    typedef K::Ray_3 Ray;
    typedef K::Triangle_3 Triangle;

    typedef CGAL::Surface_mesh<Point> Mesh;
    typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
    typedef boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;

    typedef std::list<Triangle>::iterator Iterator;
    typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
    typedef CGAL::AABB_traits<K, Primitive> Traits;
    typedef CGAL::AABB_tree<Traits> Tree;
    typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;

    struct Quad
    {
        const std::string roadID;
        const int laneID;
        double sBegin, sEnd;
        odr::Vec2D pointOnSBegin, pointOnSEnd; // must be parallel to long side
    };
}

namespace RoadRunner
{
    struct RayCastQuery
    {
        odr::Vec3D origin;
        odr::Vec3D direction;
    };

    struct RayCastResult
    {
        bool hit = false;
        //bool isJunction;
        std::string roadID;
        int lane;
        double s;
    };

    typedef uint64_t FaceIndex_t;

    class SpatialIndexer
    {
    public:
        static SpatialIndexer* Instance();

        FaceIndex_t Index(odr::Road road, odr::Lane lane, double sBegin, double sEnd);

        RayCastResult RayCast(RayCastQuery ray);

        bool UnIndex(FaceIndex_t index);

    private:
        SpatialIndexer();

        static SpatialIndexer* _instance;

        Mesh mesh;

        Tree tree;

        std::map<uint32_t, Quad> faceInfo;


    };
}