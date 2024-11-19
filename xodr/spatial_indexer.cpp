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

    SpatialIndexer::SpatialIndexer()
    {
        
    }

    uint32_t SpatialIndexer::Index(odr::Road road, odr::Lane lane, double sBegin, double sEnd)
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
        uint32_t faceID = mesh.add_face(s1t1, s1t2, s2t2, s2t1);
        faceInfo.emplace(faceID, Quad{ road.id, lane.id, sBegin, sEnd });
        tree.clear();
        tree.insert(faces(mesh).begin(), faces(mesh).end(), mesh);
        spdlog::info("Added face {}", faceID);
        return faceID;
    }

    SpatialQueryResult SpatialIndexer::RayCast(RayCastQuery ray)
    {
        SpatialQueryResult rtn;
        if (ray.direction[2] > -0.1 || ray.origin[2] < 0.1)
        {
            return rtn;
        }
        
        Ray ray_query(Point(ray.origin[0], ray.origin[1], ray.origin[2]),
            Vector(ray.direction[0], ray.direction[1], ray.direction[2]));

        Ray_intersection intersection = tree.first_intersection(ray_query);
        if (intersection.has_value())
        {
            auto faceID = intersection->second.id();
            auto info = faceInfo.at(faceID);
            return SpatialQueryResult{ true, info.roadID, info.laneID, info.sBegin };// TODO: calc s
        }

        return rtn;
    }

    bool SpatialIndexer::UnIndex(uint32_t index)
    {
        faceInfo.erase(index);
        mesh.remove_face(static_cast<face_descriptor>(index));
        return true;
    }
}