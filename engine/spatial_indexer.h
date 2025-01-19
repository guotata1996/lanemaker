#pragma once

#include "Road.h"
#include "id_generator.h"

#include <QMatrix4x4>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>

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
        std::string roadID;
        const int laneIDNormal, laneIDReversed;
        double sBegin, sEnd;
        odr::Vec2D pointOnSBegin, pointOnSEnd; // must be parallel to long side
        bool magneticArea;

        int GetLaneID()
        {
            return sBegin < sEnd ? laneIDNormal : laneIDReversed;
        }
    };
}

namespace RoadRunner
{
    typedef uint64_t FaceIndex_t;

    struct RayCastSkip
    {
        std::set<face_descriptor> fd;

        RayCastSkip() = default;

        RayCastSkip(std::set<FaceIndex_t> indice)
        {
            for (auto index : indice)
            {
                uint32_t face1ID = index >> 32;
                uint32_t face2ID = index & 0xffffffff;
                fd.emplace(face1ID);
                fd.emplace(face2ID);
            }
        }

        bool operator()(const face_descriptor& t) const
        {
            return fd.find(t) != fd.end();
        }

    };

    struct RayCastQuery
    {
        odr::Vec3D origin;
        odr::Vec3D direction;
        RayCastSkip skip;
    };

    struct RayCastResult
    {
        bool hit = false;
        odr::Vec3D hitPos;
        //bool isJunction;
        std::string roadID;
        int lane;
        double s;
    };

    class SpatialIndexer
    {
    public:
        static SpatialIndexer* Instance();

        FaceIndex_t Index(odr::Road road, odr::Lane lane, double sBegin, double sEnd);

        RayCastResult RayCast(RayCastQuery ray);

        std::vector<RayCastResult> AllOverlaps(odr::Vec3D origin, double zRange = 0.01);

        void UnIndex(FaceIndex_t index);

        void RebuildTree();

        std::map<uint32_t, Quad> faceInfo;

        static uint32_t InvalidFace;

        void Clear();

    private:
        static SpatialIndexer* _instance;

        Mesh mesh;

        Tree tree;
    };

    class SpatialIndexerDynamic
    {
    public:
        static SpatialIndexerDynamic* Instance();

        // New or update
        void Index(unsigned int id, QMatrix4x4 transform, QVector3D lwh);

        // returns -1 if no intersection
        unsigned int RayCast(odr::Vec3D origin, odr::Vec3D direction);

        void UnIndex(unsigned int id);
    private:
        static SpatialIndexerDynamic* _instance;

        std::map<unsigned int, std::vector<Triangle>> idToFaces;

        static Point ToPoint_3(QVector3D v);
    };
}