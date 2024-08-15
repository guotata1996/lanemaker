#include "RefLine.h"
#include "Geometries/RoadGeometry.h"
#include "Math.hpp"
#include "Utils.hpp"

#include <cmath>
#include <functional>
#include <iterator>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include <cassert>

#include <iostream>

namespace odr
{
double RefLine::MinGeoLength = 1e-3f;

RefLine::RefLine(std::string road_id, double length) : road_id(road_id), length(length) {}

RefLine::RefLine(const RefLine& other) : road_id(other.road_id), length(other.length), elevation_profile(other.elevation_profile)
{
    for (const auto& s0_geometry : other.s0_to_geometry)
        this->s0_to_geometry.emplace(s0_geometry.first, s0_geometry.second->clone());
}

void RefLine::reverse()
{
    decltype(s0_to_geometry) new_s0_to_geo;
    
    std::vector<double> keys = get_map_keys_sorted(s0_to_geometry);
    
    if (std::find(keys.begin(), keys.end(), length) == keys.end())
    {
        keys.push_back(length);
        std::sort(keys.begin(), keys.end());
    }
    
    for (int i = 0; i < keys.size() - 1; ++i)
    {
        if (keys[i] >= length)
            break;

        auto section = std::move(s0_to_geometry.at(keys[i]));
        section->reverse();

        double newMile = length - keys[i + 1];
        section->s0 = newMile;
        new_s0_to_geo.emplace(newMile, std::move(section));
    }

    s0_to_geometry = std::move(new_s0_to_geo);

    elevation_profile.reverse(length);
}

RefLine RefLine::split(double s) 
{
    if (s <= 2 * MinGeoLength) 
    {
        throw std::logic_error("Cannot split ref line at nearly-zero s!");
    }
    if (s >= length - 2 * MinGeoLength) 
    {
        throw std::logic_error("Cannot split ref line at nearly-length s!");
    }
    
    auto maps_keys = odr::get_map_keys_sorted(s0_to_geometry);

    // Don't RoadGeometry cut into tiny pieces
    for (const auto& key : maps_keys) 
    {
        if (std::abs(s - key) < 2 * MinGeoLength) 
        {
            s = key;
            break;
        }
    }

    // configure second
    RefLine second("", length - s);
    
    for (int keyIndex = 0; keyIndex < maps_keys.size(); ++keyIndex)
    {
        double key1 = maps_keys[keyIndex];
        double key2 = keyIndex + 1 == maps_keys.size() ? length : maps_keys[keyIndex + 1];
        
        auto&  roadGeometryPtr = s0_to_geometry.at(key1);
        if (key2 <= s) 
        {
            // Already in first s0_to_geometry, no nothing
        }
        else
        {
            auto newGeoPtr = roadGeometryPtr->clone();
            if (key1 < s && s < key2)
            {
                roadGeometryPtr->trim(s - key1);
                newGeoPtr->rebase(s - key1);
                newGeoPtr->s0 = 0;
                second.s0_to_geometry.emplace(0, std::move(newGeoPtr));
            }
            else
            {
                newGeoPtr->s0 = key1 - s;
                second.s0_to_geometry.emplace(key1 - s, std::move(newGeoPtr));
            }
        }
    }
    
    // trim first
    for (double key : maps_keys) 
    {
        if (key >= s)
            s0_to_geometry.erase(key);
    }
    length = s; 

    enforceMinGeoLength();
    second.enforceMinGeoLength();

    second.elevation_profile = elevation_profile.split(s);
    
    return second;
}

void RefLine::enforceMinGeoLength()
{
    auto keys = odr::get_map_keys(s0_to_geometry);
    for (auto it = keys.begin(); it != keys.end(); ++it) 
    {
        auto next = it;
        next++;
        if (next == keys.end())
        {
            break;
        }
        if (*next - *it < MinGeoLength) 
        {
            auto errMsg = "ref line keys too close: " + std::to_string(*it) + " & " + std::to_string(*next);
            throw std::logic_error(errMsg);
        }
    }
}

std::set<const RoadGeometry*> RefLine::get_geometries() const
{
    std::set<const RoadGeometry*> geometries;
    for (const auto& s0_geometry : this->s0_to_geometry)
        geometries.insert(s0_geometry.second.get());
    return geometries;
}

std::set<RoadGeometry*> RefLine::get_geometries()
{
    std::set<RoadGeometry*> geometries;
    for (auto& s0_geometry : this->s0_to_geometry)
        geometries.insert(s0_geometry.second.get());
    return geometries;
}

double RefLine::get_geometry_s0(const double s) const
{
    if (this->s0_to_geometry.empty())
        return NAN;
    auto target_geom_iter = this->s0_to_geometry.upper_bound(s);
    if (target_geom_iter != s0_to_geometry.begin())
        target_geom_iter--;
    return target_geom_iter->first;
}

const RoadGeometry* RefLine::get_geometry(const double s) const
{
    const double geom_s0 = this->get_geometry_s0(s);
    if (std::isnan(geom_s0))
        return nullptr;
    return this->s0_to_geometry.at(geom_s0).get();
}

RoadGeometry* RefLine::get_geometry(const double s)
{
    RoadGeometry* road_geometry = const_cast<RoadGeometry*>(static_cast<const RefLine&>(*this).get_geometry(s));
    return road_geometry;
}

Vec2D RefLine::get_xy(const double s, const double t) const 
{
    const RoadGeometry* geom = this->get_geometry(s);

    Vec2D pt_xy{0, 0};
    if (geom)
    {
        pt_xy = geom->get_xy(s);
        Vec2D fwd = geom->get_grad(s);
        Vec2D right{-fwd[1], fwd[0]};
        right = mut(t, right);
        pt_xy = add(pt_xy, right);
    }
    return pt_xy;
}

Vec3D RefLine::get_xyz(const double s, const double t) const
{
    const RoadGeometry* geom = this->get_geometry(s);

    Vec2D pt_xy = get_xy(s, t);

    return Vec3D{pt_xy[0], pt_xy[1], this->elevation_profile.get(s)};
}

Vec2D RefLine::get_grad_xy(const double s) const 
{
    const RoadGeometry* geom = this->get_geometry(s);

    Vec2D d_xy{0, 0};
    if (geom)
        d_xy = geom->get_grad(s);
    return d_xy;
}

Vec3D RefLine::get_grad(const double s) const
{
    const RoadGeometry* geom = this->get_geometry(s);

    Vec2D d_xy = get_grad_xy(s);

    return Vec3D{d_xy[0], d_xy[1], this->elevation_profile.get_grad(s)};
}

double RefLine::get_hdg(const double s) const 
{ 
    Vec2D grad = get_grad_xy(s);
    return std::atan2(grad[1], grad[0]);
}

double RefLine::match(const double x, const double y) const
{
    std::function<double(double)> f_dist = [&](const double s)
    {
        const Vec3D pt = this->get_xyz(s);
        return euclDistance(Vec2D{pt[0], pt[1]}, {x, y});
    };
    return golden_section_search<double>(f_dist, 0.0, length, 1e-2);
}

Line3D RefLine::get_line(const double s_start, const double s_end, const double eps) const
{
    std::set<double> s_vals = this->approximate_linear(eps, s_start, s_end);

    Line3D out_line;
    for (const double& s : s_vals)
        out_line.push_back(this->get_xyz(s));
    return out_line;
}

std::set<double> RefLine::approximate_linear(const double eps, const double s_start, const double s_end) const
{
    if ((s_start == s_end) || this->s0_to_geometry.empty())
        return {};

    auto s_end_geom_iter = this->s0_to_geometry.lower_bound(s_end);
    auto s_start_geom_iter = this->s0_to_geometry.upper_bound(s_start);
    if (s_start_geom_iter != s0_to_geometry.begin())
        s_start_geom_iter--;

    std::vector<double> s_vals{s_start};
    for (auto s0_geom_iter = s_start_geom_iter; s0_geom_iter != s_end_geom_iter; s0_geom_iter++)
    {
        const std::set<double> s_vals_geom = s0_geom_iter->second->approximate_linear(eps);
        if (s_vals_geom.size() < 2)
            throw std::runtime_error("expected at least two sample points");
        for (const double& s : s_vals_geom)
        {
            if (s > s_start && s < s_end)
                s_vals.push_back(s);
        }
        if (std::next(s0_geom_iter) != s_end_geom_iter)
            s_vals.pop_back();
    }

    std::set<double> s_vals_elevation = this->elevation_profile.approximate_linear(eps, s_start, s_end);
    for (const double& s : s_vals_elevation)
    {
        if (s > s_start && s < s_end)
            s_vals.push_back(s);
    }

    s_vals.push_back(s_end);

    std::set<double> s_vals_set(s_vals.begin(), s_vals.end());
    return s_vals_set;
}

} // namespace odr