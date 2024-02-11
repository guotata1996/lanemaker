#pragma once
#include "Lane.h"
#include "XmlNode.h"

#include <map>
#include <string>
#include <vector>

namespace odr
{

struct LaneSection : public XmlNode
{
    LaneSection(){};

    LaneSection(std::string road_id, double s0);

    std::vector<Lane> get_lanes() const;

    std::vector<Lane> get_sorted_driving_lanes(int8_t side); // center to rim

    int  get_lane_id(const double s, const double t) const;
    Lane get_lane(const double s, const double t) const;

    std::string         road_id = "";
    double              s0 = 0;
    std::map<int, Lane> id_to_lane;
};

} // namespace odr
