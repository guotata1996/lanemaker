#include "spatial_indexer.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/AABB_tree.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::FT FT;
typedef K::Point_3 Point;
typedef K::Vector_3 Vector;
typedef K::Ray_3 Ray;
typedef K::Triangle_3 Triangle;

namespace LM
{
    class SpatialIndexerDynamic_impl {
    public:
        // New or update
        void Index(unsigned int id, QMatrix4x4 transform, QVector3D lwh);

        // returns -1 if no intersection
        unsigned int RayCast(odr::Vec3D origin, odr::Vec3D direction);

        void UnIndex(unsigned int id);

    private:
        std::map<unsigned int, std::vector<Triangle>> idToFaces;

        static Point ToPoint_3(QVector3D v);
    };

    SpatialIndexerDynamic* SpatialIndexerDynamic::_instance = nullptr;

    SpatialIndexerDynamic* SpatialIndexerDynamic::Instance()
    {
        if (_instance == nullptr)
        {
            _instance = new SpatialIndexerDynamic();
        }
        return _instance;
    }

    SpatialIndexerDynamic::SpatialIndexerDynamic(): _impl(new SpatialIndexerDynamic_impl) {}

    void SpatialIndexerDynamic::Index(unsigned int id, QMatrix4x4 transform, QVector3D lwh) {
        _impl->Index(id, transform, lwh);
    }

    unsigned int SpatialIndexerDynamic::RayCast(odr::Vec3D origin, odr::Vec3D direction) {
        return _impl->RayCast(origin, direction);
    }

    void SpatialIndexerDynamic::UnIndex(unsigned int id) {
        _impl->UnIndex(id);
    }

    Point SpatialIndexerDynamic_impl::ToPoint_3(QVector3D v)
    {
        return Point(v.x(), v.y(), v.z());
    }

    void SpatialIndexerDynamic_impl::Index(unsigned int id, QMatrix4x4 transform, QVector3D lwh)
    {
        QVector3D whl(lwh.y(), lwh.z(), lwh.x());
        auto topLeftFront = transform.map(whl * QVector3D(0.5, 0.5, 0.5));
        auto topLeftBack = transform.map(whl * QVector3D(-0.5, 0.5, 0.5));
        auto topRightFront = transform.map(whl * QVector3D(0.5, 0.5, -0.5));
        auto topRightBack = transform.map(whl * QVector3D(-0.5, 0.5, -0.5));
        auto topLeftFrontP = ToPoint_3(topLeftFront);
        auto topLeftBackP = ToPoint_3(topLeftBack);
        auto topRightFrontP = ToPoint_3(topRightFront);
        auto topRightBackP = ToPoint_3(topRightBack);

        auto bottomLeftFront = transform.map(whl * QVector3D(0.5, -0.5, 0.5));
        auto bottomLeftBack = transform.map(whl * QVector3D(-0.5, -0.5, 0.5));
        auto bottomRightFront = transform.map(whl * QVector3D(0.5, -0.5, -0.5));
        auto bottomRightBack = transform.map(whl * QVector3D(-0.5, -0.5, -0.5));
        auto bottomLeftFrontP = ToPoint_3(bottomLeftFront);
        auto bottomLeftBackP = ToPoint_3(bottomLeftBack);
        auto bottomRightFrontP = ToPoint_3(bottomRightFront);
        auto bottomRightBackP = ToPoint_3(bottomRightBack);

        std::vector<Triangle> faces;
        // top face
        faces.emplace_back(Triangle(topLeftFrontP, topLeftBackP, topRightFrontP));
        faces.emplace_back(Triangle(topLeftBackP, topRightFrontP, topRightBackP));
        // left face
        faces.emplace_back(Triangle(topLeftFrontP, topLeftBackP, bottomLeftFrontP));
        faces.emplace_back(Triangle(topLeftBackP, bottomLeftFrontP, bottomLeftBackP));
        // right face
        faces.emplace_back(Triangle(topRightFrontP, topRightBackP, bottomRightFrontP));
        faces.emplace_back(Triangle(topRightBackP, bottomRightFrontP, bottomRightBackP));
        // front face
        faces.emplace_back(Triangle(topRightFrontP, topLeftFrontP, bottomRightFrontP));
        faces.emplace_back(Triangle(topLeftFrontP, bottomRightFrontP, bottomLeftBackP));
        idToFaces[id] = faces;
    }

    void SpatialIndexerDynamic_impl::UnIndex(unsigned int id)
    {
        idToFaces.erase(id);
    }

    unsigned int SpatialIndexerDynamic_impl::RayCast(odr::Vec3D origin, odr::Vec3D direction)
    {
        auto originP = Point(origin[0], origin[1], origin[2]);
        Ray ray_query(originP, Vector(direction[0], direction[1], direction[2]));

        double shortestDist = std::numeric_limits<float>::max();
        unsigned int result = -1;

        for (auto id_object : idToFaces)
        {
            for (auto face : id_object.second)
            {
                auto intersection = CGAL::intersection(face, ray_query);
                if (intersection.has_value())
                {
                    auto sqrDist = CGAL::squared_distance(face, originP);
                    if (sqrDist < shortestDist)
                    {
                        shortestDist = sqrDist;
                        result = id_object.first;
                    }
                }
            }
        }
        return result;
    }
}