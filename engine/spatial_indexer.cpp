#include "spatial_indexer.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/box_intersection_d.h>

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
    class AABB
    {
    public:
        typedef double NT;
        typedef unsigned long ID;

        static int dimension () {return 2;}
        ID id() const {return _id;}

        AABB(const std::shared_ptr<Quad> q, ID id=0): quad(q), _id(id)
        {
            _min[0] = std::min({q->pointS1T1[0], q->pointS2T1[0], q->pointS1T2[0], q->pointS2T2[0]});
            _min[1] = std::min({q->pointS1T1[1], q->pointS2T1[1], q->pointS1T2[1], q->pointS2T2[1]});
            _max[0] = std::max({q->pointS1T1[0], q->pointS2T1[0], q->pointS1T2[0], q->pointS2T2[0]});
            _max[1] = std::max({q->pointS1T1[1], q->pointS2T1[1], q->pointS1T2[1], q->pointS2T2[1]});
        }

        AABB& operator=(const AABB& a) {
            quad = a.quad;

            _min[0] = a._min[0];
            _min[1] = a._min[1];
            _max[0] = a._max[0];
            _max[1] = a._max[1];

            return *this;
        }

        std::weak_ptr<Quad> quad;
        const ID _id;

        double min_coord(int& dim) const { return _min[dim]; }
        double max_coord(int& dim) const { return _max[dim]; }

    private:
        double _min[2];
        double _max[2];
    };

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
        FaceIndex_t Index(const odr::Road& road, const odr::Lane& lane, double sBegin, double sEnd);

        RayCastResult RayCast(RayCastQuery ray);

        // Coarse check in 2D, fine check in 3D
        // Pass temporary (LaneProfile, RefLine) rather than indexed LM::Road
        std::vector<OverlapResult> AllOverlaps(const LaneProfile& p, const odr::RefLine& l,
            double sBegin, double sEnd, double zRange);

        void UnIndex(FaceIndex_t index);

        void RebuildTree();

        void Clear();

        std::unordered_map<uint32_t, std::shared_ptr<Quad>> faceInfo;

    private:
        // AABB Tree
        Mesh mesh;
        Tree tree;
        // box_intersection_d
        std::vector<AABB> _boxes;
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

    std::vector<OverlapResult> SpatialIndexer::AllOverlaps(
        const LaneProfile& p, const odr::RefLine& l,
        double sBegin, double sEnd, double zRange) {
        return _impl->AllOverlaps(p, l, sBegin, sEnd, zRange);
    }

    void SpatialIndexer::UnIndex(FaceIndex_t index) {
        return _impl->UnIndex(index);
    }

    void SpatialIndexer::RebuildTree() {
        _impl->RebuildTree();
    }

    std::shared_ptr<Quad> SpatialIndexer::FaceInfo(FaceIndex_t index) {
        uint32_t face1ID = index >> 32;
        uint32_t face2ID = index & 0xffffffff;
        if (face1ID != InvalidFace) {
            return _impl->faceInfo.at(face1ID);
        }
        if (face2ID != InvalidFace) {
            return _impl->faceInfo.at(face2ID);
        }
        return nullptr;
    }

    void SpatialIndexer::Clear() {
        _impl->Clear();
    }

    FaceIndex_t SpatialIndexer_impl::Index(const odr::Road& road, const odr::Lane& lane, double sBegin, double sEnd)
    {
        bool magneticArea = sBegin < 0 || sEnd > road.length;
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

        auto face = std::make_shared<Quad>(
            Quad{road.id, lane.id, laneIDWhenReversed, sBegin, sEnd,
            p1_3, p3_3, p2_3, p4_3, magneticArea});

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

        if (face1ID != SpatialIndexer::InvalidFace || face2ID != SpatialIndexer::InvalidFace) {
            _boxes.emplace_back(face);
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
                auto dir = odr::normalize(odr::to_2d(odr::sub(info->pointS2T1, info->pointS1T1)));
                auto projLength = odr::dot(dir, odr::sub(p2d, odr::to_2d(info->pointS1T1)));
                auto quadLength = odr::euclDistance(info->pointS1T1, info->pointS2T1);
                auto hitS = (projLength * info->sEnd + (quadLength - projLength) * info->sBegin) / quadLength;
                return RayCastResult{ true, p3d, info->roadID, info->LaneID(), hitS };
            }
        }

        return rtn;
    }

    std::vector<OverlapResult> SpatialIndexer_impl::AllOverlaps(
        const LaneProfile& p, const odr::RefLine& l,
        double sBegin, double sEnd, double zRange)
    {
        odr::Road odrRoad("", l.length, "-1");
        odrRoad.rr_profile = p;
        odrRoad.ref_line = odr::RefLine(l);

        odrRoad.length = l.length;
        odrRoad.rr_profile.Apply(l.length, &odrRoad);
        odrRoad.PlaceMarkings();
        odrRoad.DeriveLaneBorders();

        const auto pts = odrRoad.sample_st(sBegin, sEnd, 2.0);
        std::map<double, std::vector<double>> tForS;
        std::set<double> allSSet;
        for (auto& pt: pts) {
            allSSet.insert(pt.first);
            tForS[pt.first].push_back(pt.second);
        }

        std::vector<double> allS(allSSet.begin(), allSSet.end());

        const int sizeS = allS.size();
        std::vector<std::shared_ptr<Quad>> newQuads;

        for (int i = 0; i < std::max(0, sizeS - 1); ++i) {
            double s1 = allS[i];
            double s2 = allS[i + 1];
            const auto& row1 = tForS[s1];
            const auto& row2 = tForS[s2];

            for (int j = 0; j < std::max({(int)row1.size(), (int)row2.size(), 1}) - 1; ++j) {
                int jForRow1 = std::min(j, (int)row1.size() - 2);
                int jForRow2 = std::min(j, (int)row2.size() - 2);
                auto ptS1T1 = odrRoad.get_xyz(s1, row1[jForRow1], 0);
                auto ptS1T2 = odrRoad.get_xyz(s1, row1[jForRow1 + 1], 0);
                auto ptS2T1 = odrRoad.get_xyz(s2, row1[jForRow2], 0);
                auto ptS2T2 = odrRoad.get_xyz(s2, row2[jForRow2 + 1], 0);
                Quad q{"", 0,0,s1,s2,
                    ptS1T1, ptS2T1, ptS1T2, ptS2T2, false};
                newQuads.emplace_back(std::make_shared<Quad>(q));
            }
        }

        std::vector<AABB> newAABBs;
        newAABBs.reserve(newQuads.size());
        for (auto q: newQuads) {
            newAABBs.emplace_back(q, 1);
        }

        std::vector<OverlapResult> rtn;

        struct Callback {
            std::vector<OverlapResult>& dest;
            const double zThreshold;

            void operator()(const AABB& a, const AABB& b) {
                auto q1 = a.quad.lock();
                auto q2 = b.quad.lock();

                if (!Overlap2D(*q1, *q2))
                    return;

                if (!Overlap3D(*q1, *q2, zThreshold)) {
                    return;
                }

                dest.emplace_back(OverlapResult{
                    q2->roadID,
                    q1->LaneID(), q2->LaneID(),
                    std::min(q1->sBegin, q1->sEnd), std::max(q1->sBegin, q1->sEnd),
                    std::min(q2->sBegin, q2->sEnd), std::max(q2->sBegin, q2->sEnd)});
            }
        };

        CGAL::box_intersection_d(
            newAABBs.begin(), newAABBs.end(),
            _boxes.begin(), _boxes.end(),
            Callback{rtn, zRange});
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

        // When quads are discarded, weak_ptr stored in _boxes will become invalid
        _boxes.erase(std::remove_if(_boxes.begin(), _boxes.end(),
            [](const AABB& box){return box.quad.expired();}), _boxes.end());
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
        _boxes.clear();
    }
}