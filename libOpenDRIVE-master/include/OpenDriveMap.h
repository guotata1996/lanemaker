#pragma once
#include "Junction.h"
#include "Road.h"
#include "RoadNetworkMesh.h"
#include "RoutingGraph.h"
#include "LaneSection.h"

#include <pugixml/pugixml.hpp>

#include <map>
#include <string>
#include <vector>

namespace odr
{

class OpenDriveMap
{
public:
    OpenDriveMap();

    OpenDriveMap(const std::string& xodr_file,
                 const bool         center_map = false,
                 const bool         with_road_objects = true,
                 const bool         with_lateral_profile = true,
                 const bool         with_lane_height = true,
                 const bool         abs_z_for_for_local_road_obj_outline = false,
                 const bool         fix_spiral_edge_cases = true,
                 const bool         with_road_signals = true);

    bool LoadString(const std::string& xodr_file,
                    const bool         center_map = false,
                    const bool         with_road_objects = true,
                    const bool         with_lateral_profile = true,
                    const bool         with_lane_height = true,
                    const bool         abs_z_for_for_local_road_obj_outline = false,
                    const bool         fix_spiral_edge_cases = true,
                    const bool         with_road_signals = true);

    bool Load(const std::string& xodr_file,
              const bool         center_map = false,
              const bool         with_road_objects = true,
              const bool         with_lateral_profile = true,
              const bool         with_lane_height = true,
              const bool         abs_z_for_for_local_road_obj_outline = false,
              const bool         fix_spiral_edge_cases = true,
              const bool         with_road_signals = true);

    std::vector<Road>     get_roads() const;
    std::vector<Junction> get_junctions() const;

    RoadNetworkMesh get_road_network_mesh(const double eps) const;
    RoutingGraph    get_routing_graph() const;
    std::vector<std::tuple<LaneKey, double, LaneKey, double>> get_routes() const;
    std::map<LaneKey, std::vector<std::pair<LaneKey, double>>> get_overlap_zones() const;
    double get_lanekey_length(LaneKey) const;

    void export_file(const std::string& fpath) const; 

    std::string        proj4 = "";
    double             x_offs = 0;
    double             y_offs = 0;
    pugi::xml_document xml_doc;

    std::map<std::string, Road>     id_to_road;
    std::map<std::string, Junction> id_to_junction;

private:
    void roadNodeToXML(const odr::RoadLink& roadLink, pugi::xml_node& out) const;

};

} // namespace odr