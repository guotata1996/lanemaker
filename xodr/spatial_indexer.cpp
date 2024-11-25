#include "spatial_indexer.h"

namespace RoadRunner
{
    SpatialIndexer* SpatialIndexer::_instance = nullptr;

    SpatialIndexer* SpatialIndexer::Instance()
    {
        if (_instance == nullptr)
        {
            _instance = new SpatialIndexer();
        }
        return _instance;
    }

    FaceIndex_t SpatialIndexer::Index(odr::Road road, odr::Lane lane, double sBegin, double sEnd)
    {
        double t1 = lane.inner_border.get(sBegin);
        double t2 = lane.outer_border.get(sBegin);
        odr::Vec2D p1 = road.get_xy(sBegin, t1);
        odr::Vec2D p2 = road.get_xy(sBegin, t2);
        double h12 = road.ref_line.elevation_profile.get(sBegin);

        double t3 = lane.inner_border.get(sEnd);
        double t4 = lane.outer_border.get(sEnd);
        odr::Vec2D p3 = road.get_xy(sEnd, t3);
        odr::Vec2D p4 = road.get_xy(sEnd, t4);
        double h34 = road.ref_line.elevation_profile.get(sEnd);

        auto s1t1 = mesh.add_vertex(Point(p1[0], p1[1], h12));
        auto s1t2 = mesh.add_vertex(Point(p2[0], p2[1], h12));
        auto s2t1 = mesh.add_vertex(Point(p3[0], p3[1], h34));
        auto s2t2 = mesh.add_vertex(Point(p4[0], p4[1], h34));
        uint32_t face1ID = mesh.add_face(s1t1, s1t2, s2t1);
        uint32_t face2ID = mesh.add_face(s2t1, s1t2, s2t2);

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

        Quad face{ road.id, lane.id, laneIDWhenReversed, sBegin, sEnd, p1, p3 };

        assert(faceInfo.find(face1ID) == faceInfo.end());
        assert(faceInfo.find(face2ID) == faceInfo.end());

        faceInfo.emplace(face1ID, face);
        faceInfo.emplace(face2ID, face);

        return (static_cast<FaceIndex_t>(face1ID) << 32) | face2ID;
    }

    RayCastResult SpatialIndexer::RayCast(RayCastQuery ray, double radius)
    {
        RayCastResult rtn;
        if (ray.direction[2] > -0.1 || ray.origin[2] < 0.1)
        {
            return rtn;
        }

        std::vector<odr::Vec3D> candidateOrigins = {ray.origin};
        if (radius != 0)
        {
            odr::Vec3D o1, o2;
            odr::get_orthogonal(ray.direction, o1, o2);
            for (int angle_slice = 0; angle_slice != 12; ++angle_slice)
            {
                auto angle = 2 * M_PI / 12 * angle_slice;
                auto xOffset = odr::mut(std::cos(angle), o1);
                auto yOffset = odr::mut(std::sin(angle), o2);
                candidateOrigins.push_back(odr::add(odr::add(ray.origin, xOffset), yOffset));
            }
        }

        for (auto origin : candidateOrigins)
        {
            Ray ray_query(Point(origin[0], origin[1], origin[2]),
                Vector(ray.direction[0], ray.direction[1], ray.direction[2]));

            Ray_intersection intersection = tree.first_intersection(ray_query);
            if (intersection.has_value())
            {
                auto faceID = intersection->second.id();
                auto info = faceInfo.at(faceID);
                if (boost::get<Point>(&(intersection->first))) {
                    const Point* p = boost::get<Point>(&(intersection->first));
                    odr::Vec2D p2d{ p->x(), p->y() };
                    auto dir = odr::normalize(odr::sub(info.pointOnSEnd, info.pointOnSBegin));
                    auto projLength = odr::dot(dir, odr::sub(p2d, info.pointOnSBegin));
                    auto quadLength = odr::euclDistance(info.pointOnSBegin, info.pointOnSEnd);
                    auto hitS = (projLength * info.sEnd + (quadLength - projLength) * info.sBegin) / quadLength;
                    return RayCastResult{ true, info.roadID, info.GetLaneID(), hitS };
                }
            }
        }

        return rtn;
    }

    void SpatialIndexer::UnIndex(FaceIndex_t index)
    {
        uint32_t face1ID = index >> 32;
        uint32_t face2ID = index & 0xffffffff;
        auto nRemoved1 = faceInfo.erase(face1ID);
        auto nRemoved2 = faceInfo.erase(face2ID);
        assert(nRemoved1 == 1);
        assert(nRemoved2 == 1);
        
        mesh.remove_face(static_cast<face_descriptor>(face1ID));
        mesh.remove_face(static_cast<face_descriptor>(face2ID));
    }

    void SpatialIndexer::RebuildTree()
    {
        tree.clear();
        tree.insert(faces(mesh).begin(), faces(mesh).end(), mesh);
    }
}