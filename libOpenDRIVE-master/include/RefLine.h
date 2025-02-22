#pragma once
#include "Geometries/CubicSpline.h"
#include "Geometries/RoadGeometry.h"
#include "Math.hpp"

#include <map>
#include <memory>
#include <set>
#include <string>

namespace odr
{

struct RefLine
{
    RefLine(std::string road_id, double length);
    RefLine(const RefLine& other);
    RefLine(RefLine&& other) noexcept = default;
    RefLine& operator=(RefLine&& other) = default;
    static double MinGeoLength;

    void reverse();
    // Remain:  section between 0 and s
    // Returns: section beyond s
    RefLine split(double s);
    void    enforceMinGeoLength();

    std::set<const RoadGeometry*> get_geometries() const;
    std::set<RoadGeometry*>       get_geometries();

    double              get_geometry_s0(const double s) const;
    const RoadGeometry* get_geometry(const double s) const;
    RoadGeometry*       get_geometry(const double s);

    Vec3D            get_xyz(const double s, const double t=0) const;
    Vec2D            get_grad_xy(const double s) const;
    Vec3D            get_grad(const double s) const;
    double           get_hdg(const double s) const;
    Line3D           get_line(const double s_start, const double s_end, const double eps) const;
    double           match(const double x, const double y) const;
    std::set<double> approximate_linear(const double eps, const double s_start, const double s_end) const;

    std::string road_id = "";
    double      length = 0;
    CubicSpline elevation_profile;

    std::map<double, std::unique_ptr<RoadGeometry>> s0_to_geometry;
};

} // namespace odr