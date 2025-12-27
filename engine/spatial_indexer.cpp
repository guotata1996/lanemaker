#include "spatial_indexer.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>

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

namespace LM
{
    struct RayCastSkip_impl
    {
        std::set<face_descriptor> fd;

        RayCastSkip_impl() = default;

        RayCastSkip_impl(std::unordered_set<FaceIndex_t> indice){
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

    class SpatialIndexer_impl
    {
    public:
        FaceIndex_t Index(odr::Road road, odr::Lane lane, double sBegin, double sEnd);

        RayCastResult RayCast(RayCastQuery ray);

        std::vector<RayCastResult> AllOverlaps(odr::Vec3D origin, double zRange = 0.01);

        void UnIndex(FaceIndex_t index);

        void RebuildTree();

        void Clear();

        std::unordered_map<uint32_t, Quad> faceInfo;

    private:
        Mesh mesh;

        Tree tree;
    };


    SpatialIndexer* SpatialIndexer::_instance = nullptr;

    uint32_t SpatialIndexer::InvalidFace = 4294967295;

    SpatialIndexer* SpatialIndexer::Instance()
    {
        if (_instance == nullptr)
        {
            _instance = new SpatialIndexer();
        }
        return _instance;
    }

    SpatialIndexer::SpatialIndexer(): _impl(new SpatialIndexer_impl){}

    FaceIndex_t SpatialIndexer::Index(odr::Road road, odr::Lane lane, double sBegin, double sEnd) {
        return _impl->Index(road, lane, sBegin, sEnd);
    }

    RayCastResult SpatialIndexer::RayCast(RayCastQuery ray) {
        return _impl->RayCast(ray);
    }

    std::vector<RayCastResult> SpatialIndexer::AllOverlaps(odr::Vec3D origin, double zRange) {
        return _impl->AllOverlaps(origin, zRange);
    }

    void SpatialIndexer::UnIndex(FaceIndex_t index) {
        return _impl->UnIndex(index);
    }

    void SpatialIndexer::RebuildTree() {
        _impl->RebuildTree();
    }

    Quad& SpatialIndexer::FaceInfo(FaceIndex_t index) {
        return _impl->faceInfo.at(index);
    }

    void SpatialIndexer::Clear() {
        _impl->Clear();
    }

    FaceIndex_t SpatialIndexer_impl::Index(odr::Road road, odr::Lane lane, double sBegin, double sEnd)
    {
        bool magnetic = sBegin < 0 || sEnd > road.length;
        double t1 = lane.inner_border.get(sBegin);
        double t2 = lane.outer_border.get(sBegin);
        auto p1_3 = road.get_xyz(sBegin, t1, 0);
        odr::Vec2D p1{ p1_3[0], p1_3[1] };
        auto p2_3 = road.get_xyz(sBegin, t2, 0);
        odr::Vec2D p2{ p2_3[0], p2_3[1] };
        double h12 = p1_3[2];

        double t3 = lane.inner_border.get(sEnd);
        double t4 = lane.outer_border.get(sEnd);
        auto p3_3 = road.get_xyz(sEnd, t3, 0);
        odr::Vec2D p3{ p3_3[0], p3_3[1] };
        auto p4_3 = road.get_xyz(sEnd, t4, 0);
        odr::Vec2D p4{ p4_3[0], p4_3[1] };
        double h34 = p3_3[2];

        auto s1t1 = mesh.add_vertex(Point(p1[0], p1[1], h12));
        auto s1t2 = mesh.add_vertex(Point(p2[0], p2[1], h12));
        auto s2t1 = mesh.add_vertex(Point(p3[0], p3[1], h34));
        auto s2t2 = mesh.add_vertex(Point(p4[0], p4[1], h34));
        uint32_t face1ID = SpatialIndexer::InvalidFace;
        if (p1 != p2 && p1 != p3 && p2 != p3)
        {
            face1ID = mesh.add_face(s1t1, s1t2, s2t1);
        }

        uint32_t face2ID = SpatialIndexer::InvalidFace;
        if (p2 != p3 && p2 != p4 && p3 != p4)
        {
            face2ID = mesh.add_face(s2t1, s1t2, s2t2);
        }
        if (face2ID == face1ID)
        {
            // duplicated face
            face2ID = SpatialIndexer::InvalidFace;
        }

        bool biDirRoad = road.rr_profile.HasSide(-1) && road.rr_profile.HasSide(1);
        int laneIDWhenReversed = -lane.id;
        if (biDirRoad)
        {
            if (lane.type == "median")
            {
                assert(lane.id == 1);
                laneIDWhenReversed = 1;
            }
            else
            {
                laneIDWhenReversed = -lane.id + 1;
            }
        }

        Quad face{ road.id, lane.id, laneIDWhenReversed, sBegin, sEnd, p1, p3, magnetic };

        if (face1ID != SpatialIndexer::InvalidFace)
        {
            assert(faceInfo.find(face1ID) == faceInfo.end());
            faceInfo.emplace(face1ID, face);
        }
        if (face2ID != SpatialIndexer::InvalidFace)
        {
            assert(faceInfo.find(face2ID) == faceInfo.end());
            faceInfo.emplace(face2ID, face);
        }

        return (static_cast<FaceIndex_t>(face1ID) << 32) | face2ID;
    }

    RayCastResult SpatialIndexer_impl::RayCast(RayCastQuery ray)
    {
        RayCastResult rtn;
        if (ray.direction[2] > -0.1 || ray.origin[2] < 0.1)
        {
            return rtn;
        }

        Ray ray_query(Point(ray.origin[0], ray.origin[1], ray.origin[2]),
            Vector(ray.direction[0], ray.direction[1], ray.direction[2]));

        Ray_intersection intersection = tree.first_intersection(ray_query, RayCastSkip_impl(ray.skip));
        if (intersection.has_value())
        {
            auto faceID = intersection->second.id();
            auto info = faceInfo.at(faceID);
            if (boost::get<Point>(&(intersection->first))) {
                const Point* p = boost::get<Point>(&(intersection->first));
                odr::Vec2D p2d{ p->x(), p->y() };
                odr::Vec3D p3d{ p->x(), p->y(), p->z() };
                auto dir = odr::normalize(odr::sub(info.pointOnSEnd, info.pointOnSBegin));
                auto projLength = odr::dot(dir, odr::sub(p2d, info.pointOnSBegin));
                auto quadLength = odr::euclDistance(info.pointOnSBegin, info.pointOnSEnd);
                auto hitS = (projLength * info.sEnd + (quadLength - projLength) * info.sBegin) / quadLength;
                return RayCastResult{ true, p3d, info.roadID, info.LaneID(), hitS };
            }
        }

        return rtn;
    }

    std::vector<RayCastResult> SpatialIndexer_impl::AllOverlaps(odr::Vec3D origin, double zRange)
    {
        std::vector<RayCastResult> rtn;

        Ray ray_query(Point(origin[0], origin[1], origin[2] + zRange), Vector(0, 0, -1));
        std::list<Ray_intersection> intersections;
        try
        {
            tree.all_intersections(ray_query, std::back_inserter(intersections));
        }
        catch (CGAL::Failure_exception)
        {
            return rtn;
        }
        for (auto intersection : intersections)
        {
            if (boost::get<Point>(&(intersection->first)))
            {
                const Point* p = boost::get<Point>(&(intersection->first));
                odr::Vec3D p3d{ p->x(), p->y(), p->z() };
                if (odr::euclDistance(origin, p3d) > zRange)
                {
                    continue;
                }

                auto faceID = intersection->second.id();
                auto info = faceInfo.at(faceID);
                if (info.magneticArea)
                {
                    continue;
                }
                odr::Vec2D p2d{ p->x(), p->y() };
                auto dir = odr::normalize(odr::sub(info.pointOnSEnd, info.pointOnSBegin));
                auto projLength = odr::dot(dir, odr::sub(p2d, info.pointOnSBegin));
                auto quadLength = odr::euclDistance(info.pointOnSBegin, info.pointOnSEnd);
                auto hitS = (projLength * info.sEnd + (quadLength - projLength) * info.sBegin) / quadLength;
                hitS = std::max(std::min(info.sBegin, info.sEnd), hitS);
                hitS = std::min(std::max(info.sBegin, info.sEnd), hitS);
                RayCastResult result{ true, p3d, info.roadID, info.LaneID(), hitS };
                rtn.emplace_back(result);
            }
        }
        return rtn;
    }

    void SpatialIndexer_impl::UnIndex(FaceIndex_t index)
    {
        uint32_t face1ID = index >> 32;
        uint32_t face2ID = index & 0xffffffff;
        
        std::set<CGAL::SM_Vertex_index> vertex;

        if (face1ID != SpatialIndexer::InvalidFace)
        {
            face_descriptor f1(face1ID);
            CGAL::Vertex_around_face_iterator<Mesh> vbegin, vend;
            for (boost::tie(vbegin, vend) = vertices_around_face(mesh.halfedge(f1), mesh);
                vbegin != vend; ++vbegin) 
            {
                vertex.emplace(*vbegin);
            }

            auto nRemoved1 = faceInfo.erase(face1ID);
            assert(nRemoved1 == 1);
            mesh.remove_face(static_cast<face_descriptor>(face1ID));
        }

        if (face2ID != SpatialIndexer::InvalidFace)
        {
            face_descriptor f2(face2ID);
            CGAL::Vertex_around_face_iterator<Mesh> vbegin, vend;
            for (boost::tie(vbegin, vend) = vertices_around_face(mesh.halfedge(f2), mesh);
                vbegin != vend; ++vbegin)
            {
                vertex.emplace(*vbegin);
            }

            auto nRemoved2 = faceInfo.erase(face2ID);
            assert(nRemoved2 == 1);
            mesh.remove_face(static_cast<face_descriptor>(face2ID));
        }
        
        for (auto vID : vertex)
        {
            mesh.remove_vertex(vID);
        }
    }

    void SpatialIndexer_impl::RebuildTree()
    {
        tree.clear();
        tree.insert(faces(mesh).begin(), faces(mesh).end(), mesh);
    }

    void SpatialIndexer_impl::Clear()
    {
        mesh.clear();
        tree.clear();
        faceInfo.clear();
    }
}