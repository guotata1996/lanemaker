#pragma once
#include "Geometries/CubicSpline.h"
#include "LaneSection.h"
#include "Math.hpp"
#include "Mesh.h"
#include "RefLine.h"
#include "RoadObject.h"
#include "RoadSignal.h"
#include "XmlNode.h"
#include "road_profile.h"

#include <cstddef>
#include <map>
#include <set>
#include <vector>

namespace odr
{

struct Lane;
struct RoadMark;

struct Crossfall : public CubicSpline
{
    enum Side
    {
        Side_Both,
        Side_Left,
        Side_Right
    };

    Crossfall() = default;

    double get_crossfall(const double s, const bool on_left_side) const;

    std::map<double, Side> sides;
};

struct RoadLink : public XmlNode
{
    enum ContactPoint
    {
        ContactPoint_None,
        ContactPoint_Start,
        ContactPoint_End
    };

    enum Type
    {
        Type_None,
        Type_Road,
        Type_Junction
    };

    RoadLink() = default;
    RoadLink(std::string id, Type type, ContactPoint contact_point = ContactPoint::ContactPoint_None);

    std::string  id = "";
    Type         type = Type_None;
    ContactPoint contact_point = ContactPoint_None;
};

struct RoadNeighbor : public XmlNode
{
    RoadNeighbor(std::string id, std::string side, std::string direction);

    std::string id = "";
    std::string side = "";
    std::string direction = "";
};

struct SpeedRecord : public XmlNode
{
    SpeedRecord(std::string max, std::string unit);

    std::string max = "";
    std::string unit = "";
};

class Road : public XmlNode
{
public:
    Road(std::string id, double length, std::string junction, std::string name = "", bool left_hand_traffic = false);

    std::vector<LaneSection> get_lanesections() const;
    std::vector<RoadObject>  get_road_objects() const;
    std::vector<RoadSignal>  get_road_signals() const;

    double      get_lanesection_s0(const double s) const;
    LaneSection get_lanesection(const double s) const;

    double get_lanesection_end(const LaneSection& lanesection) const;
    double get_lanesection_end(const double lanesection_s0) const;
    double get_lanesection_length(const LaneSection& lanesection) const;
    double get_lanesection_length(const double lanesection_s0) const;

    Vec2D get_xy(const double s, const double t = 0) const;
    Vec3D get_xyz(double s, const double t, const double h, Vec3D* e_s = nullptr, Vec3D* e_t = nullptr, Vec3D* e_h = nullptr) const;
    Vec2D get_boundary_xy(int side, double s) const;
    Vec3D get_boundary_xyz(int side, double s) const;
    Vec3D get_surface_pt(double s, const double t, Vec3D* vn = nullptr) const;
    /*Uniformly sample of (s,t) pairs with max interval (min density)*/
    std::vector<std::pair<double, double>> sample_st(double sBegin, double sEnd, double interval) const;

    Line3D get_lane_border_line(const Lane& lane, const double s_start, const double s_end, const double eps, const bool outer = true) const;
    /*Ensure s match*/
    void   get_lane_border_line(const Lane& lane, const double s_start, const double s_end, const double eps, Line3D& outerOut, Line3D& innerOut) const;
    Line3D get_lane_border_line(const Lane& lane, const double eps, const bool outer = true) const;
    Line3D get_lane_center_line(const LaneKey& lane, const double travelSBegin, const double travelSEnd, const double eps) const;
    /*Returned line follows traffic direction on side*/
    Line3D get_side_border_line(const int8_t side, const double s_start, const double s_end, const bool outer, const double eps) const;
    /*Returns: left-side boundary (if exist), right-side boundary (if exist)*/
    std::pair<Line3D, Line3D> get_both_dirs_poly(const double s_start, const double s_end, const double eps) const;
    /*Always returns left & right start at 0, end at length*/
    Line3D get_road_boundary(int side, const double eps) const;
    std::pair<Line3D, Line3D> get_lane_marking_line(const Lane& lane, const double s_start, const double s_end, const double width, const double eps) const;

    Mesh3D get_lane_mesh(
        const Lane& lane, const double s_start, const double s_end, const double eps, std::vector<uint32_t>* outline_indices = nullptr) const;
    Mesh3D get_lane_mesh(const Lane& lane, const double eps, std::vector<uint32_t>* outline_indices = nullptr) const;

    Mesh3D get_roadmark_mesh(const Lane& lane, const RoadMark& roadmark, const double eps) const;
    Mesh3D get_road_signal_mesh(const RoadSignal& road_signal) const;
    Mesh3D get_road_object_mesh(const RoadObject& road_object, const double eps) const;

    std::set<double>
    approximate_lane_border_linear(const Lane& lane, const double s_start, const double s_end, const double eps, const bool outer = true) const;
    std::set<double> approximate_lane_border_linear(const Lane& lane, const double eps, const bool outer = true) const;

    void DeriveLaneBorders();

    void PlaceMarkings();
    bool HideBorderMarkingForDJ(odr::RoadLink::ContactPoint, int side, double untilS);
    double EnableBorderMarking(odr::RoadLink::ContactPoint, int side); // Returns untilS that need to be updated

    void ToggleStopLine(odr::RoadLink::ContactPoint, bool enable);
    void UpdateArrowMarkings(odr::RoadLink::ContactPoint, std::map<int, uint8_t> laneToArrow);

    double      length = 0;
    std::string id = "";
    std::string junction = "";
    std::string name = "";
    bool        left_hand_traffic = false;

    RoadLink                  predecessor;
    RoadLink                  successor;
    std::vector<RoadNeighbor> neighbors;

    CubicSpline lane_offset;
    CubicSpline superelevation;
    Crossfall   crossfall;
    RefLine     ref_line;

    std::map<double, LaneSection>     s_to_lanesection;
    std::map<double, std::string>     s_to_type;
    std::map<double, SpeedRecord>     s_to_speed;
    std::map<std::string, RoadObject> id_to_object; // "1","2" used for pred/succ stop lines
    std::map<std::string, RoadSignal> id_to_signal;
    LM::LaneProfile           rr_profile;

    std::map<std::pair<odr::RoadLink::ContactPoint, int>, double> boundaryHide;
};

} // namespace odr
