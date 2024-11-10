#include "OpenDriveMap.h"
#include "Geometries/Arc.h"
#include "Geometries/CubicSpline.h"
#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
#include "Geometries/RoadGeometry.h"
#include "Geometries/Spiral.h"
#include "Junction.h"
#include "Lane.h"
#include "LaneSection.h"
#include "LaneValidityRecord.h"
#include "Math.hpp"
#include "RefLine.h"
#include "Road.h"
#include "RoadMark.h"
#include "RoadObject.h"
#include "RoadSignal.h"
#include "Utils.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <iterator>
#include <memory>
#include <set>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <cstring>
#include <type_traits>
#include <utility>
#include <vector>
#include <fstream>

namespace odr
{
std::vector<LaneValidityRecord> extract_lane_validity_records(const pugi::xml_node& xml_node)
{
    std::vector<LaneValidityRecord> lane_validities;
    for (const auto& validity_node : xml_node.children("validity"))
    {
        LaneValidityRecord lane_validity{validity_node.attribute("fromLane").as_int(INT_MIN), validity_node.attribute("toLane").as_int(INT_MAX)};
        lane_validity.xml_node = validity_node;

        // fromLane should not be greater than toLane, since the standard defines them as follows:
        // fromLane - the minimum ID of lanes for which the object is valid
        // toLane - the maximum ID of lanes for which the object is valid
        // If we find such a violation, we set both IDs to 0 which means the
        // object is only applicable to the centerlane.
        CHECK_AND_REPAIR(
            lane_validity.from_lane <= lane_validity.to_lane, "lane_validity::from_lane > lane_validity.to_lane", lane_validity.from_lane = 0;
            lane_validity.to_lane = 0)

        lane_validities.push_back(std::move(lane_validity));
    }
    return lane_validities;
}

OpenDriveMap::OpenDriveMap() {}

OpenDriveMap::OpenDriveMap(const std::string& xodr_file,
                           const bool         center_map,
                           const bool         with_road_objects,
                           const bool         with_lateral_profile,
                           const bool         with_lane_height,
                           const bool         abs_z_for_for_local_road_obj_outline,
                           const bool         fix_spiral_edge_cases,
                           const bool         with_road_signals)
    //:xodr_file(xodr_file)
{
    Load(xodr_file,
         center_map,
         with_road_objects,
         with_lateral_profile,
         with_lane_height,
         abs_z_for_for_local_road_obj_outline,
         fix_spiral_edge_cases,
         with_road_signals);
}


bool OpenDriveMap::LoadString(const std::string& xodr_str,
          const bool         center_map,
          const bool         with_road_objects,
          const bool         with_lateral_profile,
          const bool         with_lane_height,
          const bool         abs_z_for_for_local_road_obj_outline,
          const bool         fix_spiral_edge_cases,
          const bool         with_road_signals)
{
    id_to_road.clear();
    id_to_junction.clear();
    bool supported = true;

    pugi::xml_parse_result result = this->xml_doc.load_string(xodr_str.c_str());
    if (!result)
        printf("Err{} %s\n", result.description());

    pugi::xml_node odr_node = this->xml_doc.child("OpenDRIVE");

    if (auto geoReference_node = odr_node.child("header").child("geoReference"))
        this->proj4 = geoReference_node.text().as_string("");

    std::size_t cnt = 1;
    if (center_map)
    {
        for (pugi::xml_node road_node : odr_node.children("road"))
        {
            for (pugi::xml_node geometry_hdr_node : road_node.child("planView").children("geometry"))
            {
                const double x0 = geometry_hdr_node.attribute("x").as_double(0.0);
                this->x_offs = this->x_offs + ((x0 - this->x_offs) / cnt);
                const double y0 = geometry_hdr_node.attribute("y").as_double(0.0);
                this->y_offs = this->y_offs + ((y0 - this->y_offs) / cnt);
                cnt++;
            }
        }
    }

    for (pugi::xml_node junction_node : odr_node.children("junction"))
    {
        /* make junction */
        const std::string junction_id = junction_node.attribute("id").as_string("");
        const JunctionType _type = strcmp(junction_node.attribute("type").as_string(""), "direct") == 0 ? JunctionType::Direct : JunctionType::Common;
        Junction& junction =
            this->id_to_junction.insert({junction_id, 
                Junction(junction_node.attribute("name").as_string(""), 
                    junction_id, _type)}).first->second;
        junction.xml_node = junction_node;

        for (pugi::xml_node connection_node : junction_node.children("connection"))
        {
            std::string contact_point_str = connection_node.attribute("contactPoint").as_string("");
            CHECK_AND_REPAIR(contact_point_str == "start" || contact_point_str == "end",
                             "Junction::Connection::contactPoint invalid value",
                             contact_point_str = "start"); // default to start
            const JunctionConnection::ContactPoint junction_conn_contact_point =
                (contact_point_str == "start") ? JunctionConnection::ContactPoint_Start : JunctionConnection::ContactPoint_End;

            std::string interface_contact_point_str = connection_node.attribute("interfaceProviderContactPoint").as_string("");

            JunctionConnection::ContactPoint junction_interface_contact_point = JunctionConnection::ContactPoint_None;
            if (interface_contact_point_str == "start") 
            {
                junction_interface_contact_point = JunctionConnection::ContactPoint_Start;
            }
            if (interface_contact_point_str == "end") 
            {
                junction_interface_contact_point = JunctionConnection::ContactPoint_End;
            }

            std::set<int> signalPhases;
            for (auto child_phase: connection_node.children("signalPhase"))
            {
                signalPhases.emplace(child_phase.attribute("id").as_int(-1));
            }
            const std::string   junction_connection_id = connection_node.attribute("id").as_string("");
            JunctionConnection& junction_connection = junction.id_to_connection
                                                          .insert({junction_connection_id,
                                                                   JunctionConnection(junction_connection_id,
                                                                                      connection_node.attribute("incomingRoad").as_string(""),
                                                                                      connection_node.attribute(_type == JunctionType::Common ? 
                                                                                          "connectingRoad" : "linkedRoad").as_string(""),
                                                                                      junction_conn_contact_point,
                                                                                      signalPhases,
                                                                                      junction_interface_contact_point)})
                                                          .first->second;

            for (pugi::xml_node lane_link_node : connection_node.children("laneLink"))
            {
                JunctionLaneLink lane_link(lane_link_node.attribute("from").as_int(0), 
                    lane_link_node.attribute("to").as_int(0), lane_link_node.attribute("overlapZone").as_float(0));
                junction_connection.lane_links.insert(lane_link);
            }
        }

        const std::size_t num_conns = junction.id_to_connection.size();
        CHECK(num_conns > 0, "Junction::connections == 0");
        if (num_conns < 1)
            continue;

        for (pugi::xml_node priority_node : junction_node.children("priority"))
        {
            JunctionPriority junction_priority(priority_node.attribute("high").as_string(""), priority_node.attribute("low").as_string(""));
            junction.priorities.insert(junction_priority);
        }

        for (pugi::xml_node controller_node : junction_node.children("controller"))
        {
            const std::string junction_controller_id = controller_node.attribute("id").as_string("");
            junction.id_to_controller.insert({junction_controller_id,
                                              JunctionController(junction_controller_id,
                                                                 controller_node.attribute("type").as_string(""),
                                                                 controller_node.attribute("sequence").as_uint(0))});
        }

        if (junction_node.child("boundary"))
        {
            for (auto segment_node : junction_node.child("boundary").children("segment"))
            {
                odr::BoundarySegment segment;
                segment.road = segment_node.attribute("roadID").as_string();
                segment.side = strcmp(segment_node.attribute("side").as_string(), "left") == 0 ? 1 : -1;
                segment.sBegin = segment_node.attribute("sStart").as_double();
                segment.sEnd = segment_node.attribute("sEnd").as_double();
                segment.type = strcmp(segment_node.attribute("type").as_string(), "lane") == 0 ? 
                    BoundarySegmentType::Lane : BoundarySegmentType::Joint;
                auto str = segment_node.attribute("side").as_string();
                junction.boundary.push_back(segment);
            }
        }
    }

    for (pugi::xml_node road_node : odr_node.children("road"))
    {
        /* make road */
        std::string road_id = road_node.attribute("id").as_string("");
        CHECK_AND_REPAIR(this->id_to_road.find(road_id) == this->id_to_road.end(),
                         (std::string("road::id already exists - ") + road_id).c_str(),
                         road_id = road_id + std::string("_dup"));

        std::string rule_str = std::string(road_node.attribute("rule").as_string("RHT"));
        std::transform(rule_str.begin(), rule_str.end(), rule_str.begin(), [](unsigned char c) { return std::tolower(c); });
        const bool is_left_hand_traffic = (rule_str == "lht");

        Road& road = this->id_to_road
                         .insert({road_id,
                                  Road(road_id,
                                       road_node.attribute("length").as_double(0.0),
                                       road_node.attribute("junction").as_string(""),
                                       road_node.attribute("name").as_string(""),
                                       is_left_hand_traffic)})
                         .first->second;
        road.xml_node = road_node;

        CHECK_AND_REPAIR(road.length >= 0, "road::length < 0", road.length = 0);

        /* parse road links */
        for (bool is_predecessor : {true, false})
        {
            pugi::xml_node road_link_node =
                is_predecessor ? road_node.child("link").child("predecessor") : road_node.child("link").child("successor");
            if (road_link_node)
            {
                RoadLink& link = is_predecessor ? road.predecessor : road.successor;
                link.id = road_link_node.attribute("elementId").as_string("");

                std::string type_str = road_link_node.attribute("elementType").as_string("");
                CHECK_AND_REPAIR(type_str == "road" || type_str == "junction",
                                 "Road::Succ/Predecessor::Link::elementType invalid type",
                                 type_str = "road"); // default to road
                link.type = (type_str == "road") ? RoadLink::Type_Road : RoadLink::Type_Junction;

                if (link.type == RoadLink::Type_Road)
                {
                    // junction connection has no contact point
                    std::string contact_point_str = road_link_node.attribute("contactPoint").as_string("");
                    CHECK_AND_REPAIR(contact_point_str == "start" || contact_point_str == "end",
                                     "Road::Succ/Predecessor::Link::contactPoint invalid type",
                                     contact_point_str = "start"); // default to start
                    link.contact_point = (contact_point_str == "start") ? RoadLink::ContactPoint_Start : RoadLink::ContactPoint_End;
                }

                link.xml_node = road_link_node;
            }
        }

        /* parse road neighbors */
        for (pugi::xml_node road_neighbor_node : road_node.child("link").children("neighbor"))
        {
            const std::string road_neighbor_id = road_neighbor_node.attribute("elementId").as_string("");
            const std::string road_neighbor_side = road_neighbor_node.attribute("side").as_string("");
            const std::string road_neighbor_direction = road_neighbor_node.attribute("direction").as_string("");
            RoadNeighbor      road_neighbor(road_neighbor_id, road_neighbor_side, road_neighbor_direction);
            road_neighbor.xml_node = road_neighbor_node;
            road.neighbors.push_back(road_neighbor);
        }

        /* parse road type and speed */
        for (pugi::xml_node road_type_node : road_node.children("type"))
        {
            double      s = road_type_node.attribute("s").as_double(0.0);
            std::string type = road_type_node.attribute("type").as_string("");

            CHECK_AND_REPAIR(s >= 0, "road::type::s < 0", s = 0);

            road.s_to_type[s] = type;
            if (pugi::xml_node node = road_type_node.child("speed"))
            {
                const std::string speed_record_max = node.attribute("max").as_string("");
                const std::string speed_record_unit = node.attribute("unit").as_string("");
                SpeedRecord       speed_record(speed_record_max, speed_record_unit);
                speed_record.xml_node = node;
                road.s_to_speed.insert({s, speed_record});
            }
        }

        /* make ref_line - parse road geometries */
        for (pugi::xml_node geometry_hdr_node : road_node.child("planView").children("geometry"))
        {
            double s0 = geometry_hdr_node.attribute("s").as_double(0.0);
            double x0 = geometry_hdr_node.attribute("x").as_double(0.0) - this->x_offs;
            double y0 = geometry_hdr_node.attribute("y").as_double(0.0) - this->y_offs;
            double hdg0 = geometry_hdr_node.attribute("hdg").as_double(0.0);
            double length = geometry_hdr_node.attribute("length").as_double(0.0);

            CHECK_AND_REPAIR(s0 >= 0, "road::planView::geometry::s < 0", s0 = 0);
            CHECK_AND_REPAIR(length >= 0, "road::planView::geometry::length < 0", length = 0);

            pugi::xml_node geometry_node = geometry_hdr_node.first_child();
            std::string    geometry_type = geometry_node.name();
            if (geometry_type == "line")
            {
                road.ref_line.s0_to_geometry[s0] = std::make_unique<Line>(s0, x0, y0, hdg0, length);
            }
            else if (geometry_type == "spiral")
            {
                double curv_start = geometry_node.attribute("curvStart").as_double(0.0);
                double curv_end = geometry_node.attribute("curvEnd").as_double(0.0);
                if (!fix_spiral_edge_cases)
                {
                    road.ref_line.s0_to_geometry[s0] = std::make_unique<Spiral>(s0, x0, y0, hdg0, length, curv_start, curv_end);
                }
                else
                {
                    if (std::abs(curv_start) < 1e-6 && std::abs(curv_end) < 1e-6)
                    {
                        // In effect a line
                        road.ref_line.s0_to_geometry[s0] = std::make_unique<Line>(s0, x0, y0, hdg0, length);
                    }
                    else if (std::abs(curv_end - curv_start) < 1e-6)
                    {
                        // In effect an arc
                        road.ref_line.s0_to_geometry[s0] = std::make_unique<Arc>(s0, x0, y0, hdg0, length, curv_start);
                    }
                    else
                    {
                        // True spiral
                        road.ref_line.s0_to_geometry[s0] = std::make_unique<Spiral>(s0, x0, y0, hdg0, length, curv_start, curv_end);
                    }
                }
            }
            else if (geometry_type == "arc")
            {
                double curvature = geometry_node.attribute("curvature").as_double(0.0);
                road.ref_line.s0_to_geometry[s0] = std::make_unique<Arc>(s0, x0, y0, hdg0, length, curvature);
            }
            else if (geometry_type == "paramPoly3")
            {
                double aU = geometry_node.attribute("aU").as_double(0.0);
                double bU = geometry_node.attribute("bU").as_double(0.0);
                double cU = geometry_node.attribute("cU").as_double(0.0);
                double dU = geometry_node.attribute("dU").as_double(0.0);
                double aV = geometry_node.attribute("aV").as_double(0.0);
                double bV = geometry_node.attribute("bV").as_double(0.0);
                double cV = geometry_node.attribute("cV").as_double(0.0);
                double dV = geometry_node.attribute("dV").as_double(0.0);

                bool pRange_normalized = true;
                if (geometry_node.attribute("pRange") || geometry_hdr_node.attribute("pRange"))
                {
                    std::string pRange_str = geometry_node.attribute("pRange") ? geometry_node.attribute("pRange").as_string("")
                                                                               : geometry_hdr_node.attribute("pRange").as_string("");
                    std::transform(pRange_str.begin(), pRange_str.end(), pRange_str.begin(), [](unsigned char c) { return std::tolower(c); });
                    if (pRange_str == "arclength")
                        pRange_normalized = false;
                }
                road.ref_line.s0_to_geometry[s0] =
                    std::make_unique<ParamPoly3>(s0, x0, y0, hdg0, length, aU, bU, cU, dU, aV, bV, cV, dV, pRange_normalized);
            }
            else
            {
                printf("Could not parse %s\n", geometry_type.c_str());
                continue;
            }

            road.ref_line.s0_to_geometry.at(s0)->xml_node = geometry_node;
        }

        std::map<std::string /*x path query*/, CubicSpline&> cubic_spline_fields{{".//elevationProfile//elevation", road.ref_line.elevation_profile},
                                                                                 {".//lanes//laneOffset", road.lane_offset}};

        if (with_lateral_profile)
            cubic_spline_fields.insert({".//lateralProfile//superelevation", road.superelevation});

        /* parse elevation profiles, lane offsets, superelevation */
        for (auto entry : cubic_spline_fields)
        {
            /* handle splines not starting at s=0, assume value 0 until start */
            entry.second.s0_to_poly[0.0] = Poly3(0.0, 0.0, 0.0, 0.0, 0.0);

            pugi::xpath_node_set nodes = road_node.select_nodes(entry.first.c_str());
            for (pugi::xpath_node node : nodes)
            {
                double s0 = node.node().attribute("s").as_double(0.0);
                double a = node.node().attribute("a").as_double(0.0);
                double b = node.node().attribute("b").as_double(0.0);
                double c = node.node().attribute("c").as_double(0.0);
                double d = node.node().attribute("d").as_double(0.0);

                CHECK_AND_REPAIR(s0 >= 0, (entry.first + "::s < 0").c_str(), s0 = 0);

                entry.second.s0_to_poly[s0] = Poly3(s0, a, b, c, d);
            }
        }

        /* parse crossfall - has extra attribute side */
        if (with_lateral_profile)
        {
            for (pugi::xml_node crossfall_node : road_node.child("lateralProfile").children("crossfall"))
            {
                double s0 = crossfall_node.attribute("s").as_double(0.0);
                double a = crossfall_node.attribute("a").as_double(0.0);
                double b = crossfall_node.attribute("b").as_double(0.0);
                double c = crossfall_node.attribute("c").as_double(0.0);
                double d = crossfall_node.attribute("d").as_double(0.0);

                CHECK_AND_REPAIR(s0 >= 0, "road::lateralProfile::crossfall::s < 0", s0 = 0);

                Poly3 crossfall_poly(s0, a, b, c, d);
                road.crossfall.s0_to_poly[s0] = crossfall_poly;
                if (pugi::xml_attribute side = crossfall_node.attribute("side"))
                {
                    std::string side_str = side.as_string("");
                    std::transform(side_str.begin(), side_str.end(), side_str.begin(), [](unsigned char c) { return std::tolower(c); });
                    if (side_str == "left")
                        road.crossfall.sides[s0] = Crossfall::Side_Left;
                    else if (side_str == "right")
                        road.crossfall.sides[s0] = Crossfall::Side_Right;
                    else
                        road.crossfall.sides[s0] = Crossfall::Side_Both;
                }
            }

            /* check for lateralProfile shape - not implemented yet */
            if (road_node.child("lateralProfile").child("shape"))
            {
                printf("Lateral Profile Shape not supported\n");
            }
        }

        /* parse road lane sections and lanes */
        for (pugi::xml_node lanesection_node : road_node.child("lanes").children("laneSection"))
        {
            const double s0 = lanesection_node.attribute("s").as_double(0.0);
            LaneSection& lanesection = road.s_to_lanesection.insert({s0, LaneSection(road_id, s0)}).first->second;
            lanesection.xml_node = lanesection_node;

            for (pugi::xpath_node lane_xpath_node : lanesection_node.select_nodes(".//lane"))
            {
                pugi::xml_node lane_node = lane_xpath_node.node();
                const int      lane_id = lane_node.attribute("id").as_int(0);

                Lane& lane =
                    lanesection.id_to_lane
                        .insert({lane_id,
                                 Lane(road_id, s0, lane_id, lane_node.attribute("level").as_bool(false), lane_node.attribute("type").as_string(""))})
                        .first->second;

                if (pugi::xml_node node = lane_node.child("link").child("predecessor"))
                    lane.predecessor = node.attribute("id").as_int(0);
                if (pugi::xml_node node = lane_node.child("link").child("successor"))
                    lane.successor = node.attribute("id").as_int(0);
                lane.xml_node = lane_node;

                for (pugi::xml_node lane_width_node : lane_node.children("width"))
                {
                    double s_offset = lane_width_node.attribute("sOffset").as_double(0.0);
                    double a = lane_width_node.attribute("a").as_double(0.0);
                    double b = lane_width_node.attribute("b").as_double(0.0);
                    double c = lane_width_node.attribute("c").as_double(0.0);
                    double d = lane_width_node.attribute("d").as_double(0.0);

                    CHECK_AND_REPAIR(s_offset >= 0, "lane::width::sOffset < 0", s_offset = 0);
                    lane.lane_width.s0_to_poly[s0 + s_offset] = Poly3(s0 + s_offset, a, b, c, d);
                }

                if (with_lane_height)
                {
                    for (pugi::xml_node lane_height_node : lane_node.children("height"))
                    {
                        double s_offset = lane_height_node.attribute("sOffset").as_double(0.0);
                        double inner = lane_height_node.attribute("inner").as_double(0.0);
                        double outer = lane_height_node.attribute("outer").as_double(0.0);

                        CHECK_AND_REPAIR(s_offset >= 0, "lane::height::sOffset < 0", s_offset = 0);
                        lane.s_to_height_offset.insert({s0 + s_offset, HeightOffset(inner, outer)});
                    }
                }

                for (pugi::xml_node roadmark_node : lane_node.children("roadMark"))
                {
                    RoadMarkGroup roadmark_group(road_id,
                                                 s0,
                                                 lane_id,
                                                 roadmark_node.attribute("width").as_double(-1),
                                                 roadmark_node.attribute("height").as_double(0),
                                                 roadmark_node.attribute("sOffset").as_double(0),
                                                 roadmark_node.attribute("type").as_string("none"),
                                                 roadmark_node.attribute("weight").as_string("standard"),
                                                 roadmark_node.attribute("color").as_string("standard"),
                                                 roadmark_node.attribute("material").as_string("standard"),
                                                 roadmark_node.attribute("laneChange").as_string("both"));
                    roadmark_group.xml_node = roadmark_node;

                    CHECK_AND_REPAIR(roadmark_group.s_offset >= 0, "lane::roadMark::sOffset < 0", roadmark_group.s_offset = 0);
                    const double roadmark_group_s0 = s0 + roadmark_group.s_offset;

                    if (pugi::xml_node roadmark_type_node = roadmark_node.child("type"))
                    {
                        const std::string name = roadmark_type_node.attribute("name").as_string("");
                        const double      line_width_1 = roadmark_type_node.attribute("width").as_double(-1);

                        for (pugi::xml_node roadmarks_line_node : roadmark_type_node.children("line"))
                        {
                            const double line_width_0 = roadmarks_line_node.attribute("width").as_double(-1);
                            const double roadmark_width = line_width_0 < 0 ? line_width_1 : line_width_0;

                            RoadMarksLine roadmarks_line(road_id,
                                                         s0,
                                                         lane_id,
                                                         roadmark_group_s0,
                                                         roadmark_width,
                                                         roadmarks_line_node.attribute("length").as_double(0),
                                                         roadmarks_line_node.attribute("space").as_double(0),
                                                         roadmarks_line_node.attribute("tOffset").as_double(0),
                                                         roadmarks_line_node.attribute("sOffset").as_double(0),
                                                         name,
                                                         roadmarks_line_node.attribute("rule").as_string("none"));
                            roadmarks_line.xml_node = roadmarks_line_node;

                            CHECK_AND_REPAIR(roadmarks_line.length >= 0, "roadMark::type::line::length < 0", roadmarks_line.length = 0);
                            CHECK_AND_REPAIR(roadmarks_line.space >= 0, "roadMark::type::line::space < 0", roadmarks_line.space = 0);
                            CHECK_AND_REPAIR(roadmarks_line.s_offset >= 0, "roadMark::type::line::sOffset < 0", roadmarks_line.s_offset = 0);

                            roadmark_group.roadmark_lines.emplace(std::move(roadmarks_line));
                        }
                    }

                    lane.roadmark_groups.emplace(std::move(roadmark_group));
                }
            }

            /* derive lane borders from lane widths */
            auto id_lane_iter0 = lanesection.id_to_lane.find(0);
            if (id_lane_iter0 == lanesection.id_to_lane.end())
                throw std::runtime_error("lane section does not have lane #0");

            /* iterate from id #0 towards +inf */
            auto id_lane_iter1 = std::next(id_lane_iter0);
            for (auto iter = id_lane_iter1; iter != lanesection.id_to_lane.end(); iter++)
            {
                if (iter == id_lane_iter1)
                {
                    iter->second.outer_border = iter->second.lane_width;
                }
                else
                {
                    iter->second.inner_border = std::prev(iter)->second.outer_border;
                    iter->second.outer_border = std::prev(iter)->second.outer_border.add(iter->second.lane_width);
                }
            }

            /* iterate from id #0 towards -inf */
            std::map<int, Lane>::reverse_iterator r_id_lane_iter_1(id_lane_iter0);
            for (auto r_iter = r_id_lane_iter_1; r_iter != lanesection.id_to_lane.rend(); r_iter++)
            {
                if (r_iter == r_id_lane_iter_1)
                {
                    r_iter->second.outer_border = r_iter->second.lane_width.negate();
                }
                else
                {
                    r_iter->second.inner_border = std::prev(r_iter)->second.outer_border;
                    r_iter->second.outer_border = std::prev(r_iter)->second.outer_border.add(r_iter->second.lane_width.negate());
                }
            }

            for (auto& id_lane : lanesection.id_to_lane)
            {
                id_lane.second.inner_border = id_lane.second.inner_border.add(road.lane_offset);
                id_lane.second.outer_border = id_lane.second.outer_border.add(road.lane_offset);
            }
        }

        pugi::xml_node customProfile_node = road_node.child("roadRunnerProfile");
        if (customProfile_node) 
        {
            pugi::xml_node leftProfile = customProfile_node.child("left");
            if (leftProfile) 
            {
                for (auto sectionNode : leftProfile.children("section")) 
                {
                    auto s = sectionNode.attribute("type_s").as_uint();
                    auto laneCount = sectionNode.attribute("laneCount").as_int();
                    auto offsetX2 = sectionNode.attribute("offsetX2").as_int();
                    RoadRunner::LanePlan profile{offsetX2, laneCount};
                    road.rr_profile.leftPlans.emplace(s, profile);
                }
            }
            
            pugi::xml_node rightProfile = customProfile_node.child("right");
            if (rightProfile) 
            {
                for (auto sectionNode : rightProfile.children("section"))
                {
                    auto s = sectionNode.attribute("type_s").as_uint();
                    auto laneCount = sectionNode.attribute("laneCount").as_int();
                    auto offsetX2 = sectionNode.attribute("offsetX2").as_int();
                    RoadRunner::LanePlan profile{offsetX2, laneCount};
                    road.rr_profile.rightPlan.emplace(s, profile);
                }
            }
        }
        else
        {
            supported = false;
        }

        pugi::xml_node boundaryHide_node = road_node.child("roadRunnerBoundaryHide");
        if (boundaryHide_node)
        {
            for (auto detail_node : boundaryHide_node.children())
            {
                odr::RoadLink::ContactPoint c = detail_node.attribute("contactPoint").as_string() == "start" ?
                    odr::RoadLink::ContactPoint_Start : odr::RoadLink::ContactPoint_End;
                int side = strcmp(detail_node.attribute("side").as_string(), "left") == 0 ? 1 : -1;
                double length = detail_node.attribute("s").as_double();
                road.boundaryHide.emplace(std::make_pair(c, side), length);
            }
        }

        /* parse road objects */
        if (with_road_objects)
        {
            const RoadObjectCorner::Type default_local_outline_type =
                abs_z_for_for_local_road_obj_outline ? RoadObjectCorner::Type_Local_AbsZ : RoadObjectCorner::Type_Local_RelZ;

            for (pugi::xml_node object_node : road_node.child("objects").children("object"))
            {
                std::string road_object_id = object_node.attribute("id").as_string("");
                CHECK_AND_REPAIR(road.id_to_object.find(road_object_id) == road.id_to_object.end(),
                                 (std::string("object::id already exists - ") + road_object_id).c_str(),
                                 road_object_id = road_object_id + std::string("_dup"));

                const bool  is_dynamic_object = std::string(object_node.attribute("dynamic").as_string("no")) == "yes" ? true : false;
                RoadObject& road_object = road.id_to_object
                                              .insert({road_object_id,
                                                       RoadObject(road_id,
                                                                  road_object_id,
                                                                  object_node.attribute("s").as_double(0),
                                                                  object_node.attribute("t").as_double(0),
                                                                  object_node.attribute("zOffset").as_double(0),
                                                                  object_node.attribute("length").as_double(0),
                                                                  object_node.attribute("validLength").as_double(0),
                                                                  object_node.attribute("width").as_double(0),
                                                                  object_node.attribute("radius").as_double(0),
                                                                  object_node.attribute("height").as_double(0),
                                                                  object_node.attribute("hdg").as_double(0),
                                                                  object_node.attribute("pitch").as_double(0),
                                                                  object_node.attribute("roll").as_double(0),
                                                                  object_node.attribute("type").as_string(""),
                                                                  object_node.attribute("name").as_string(""),
                                                                  object_node.attribute("orientation").as_string(""),
                                                                  object_node.attribute("subtype").as_string(""),
                                                                  is_dynamic_object)})
                                              .first->second;
                road_object.xml_node = object_node;

                CHECK_AND_REPAIR(road_object.s0 >= 0, "object::s < 0", road_object.s0 = 0);
                CHECK_AND_REPAIR(road_object.valid_length >= 0, "object::validLength < 0", road_object.valid_length = 0);
                CHECK_AND_REPAIR(road_object.length >= 0, "object::length < 0", road_object.length = 0);
                CHECK_AND_REPAIR(road_object.width >= 0, "object::width < 0", road_object.width = 0);
                CHECK_AND_REPAIR(road_object.radius >= 0, "object::radius < 0", road_object.radius = 0);

                for (pugi::xml_node repeat_node : object_node.children("repeat"))
                {
                    RoadObjectRepeat road_object_repeat(repeat_node.attribute("s").as_double(NAN),
                                                        repeat_node.attribute("length").as_double(0),
                                                        repeat_node.attribute("distance").as_double(0),
                                                        repeat_node.attribute("tStart").as_double(NAN),
                                                        repeat_node.attribute("tEnd").as_double(NAN),
                                                        repeat_node.attribute("widthStart").as_double(NAN),
                                                        repeat_node.attribute("widthEnd").as_double(NAN),
                                                        repeat_node.attribute("heightStart").as_double(NAN),
                                                        repeat_node.attribute("heightEnd").as_double(NAN),
                                                        repeat_node.attribute("zOffsetStart").as_double(NAN),
                                                        repeat_node.attribute("zOffsetEnd").as_double(NAN));
                    road_object_repeat.xml_node = repeat_node;

                    CHECK_AND_REPAIR(
                        std::isnan(road_object_repeat.s0) || road_object_repeat.s0 >= 0, "object::repeat::s < 0", road_object_repeat.s0 = 0);
                    CHECK_AND_REPAIR(std::isnan(road_object_repeat.width_start) || road_object_repeat.width_start >= 0,
                                     "object::repeat::widthStart < 0",
                                     road_object_repeat.width_start = 0);
                    CHECK_AND_REPAIR(std::isnan(road_object_repeat.width_end) || road_object_repeat.width_end >= 0,
                                     "object::repeat::widthStart < 0",
                                     road_object_repeat.width_end = 0);
                    CHECK_AND_REPAIR(road_object_repeat.length >= 0, "object::repeat::length < 0", road_object_repeat.length = 0);
                    CHECK_AND_REPAIR(road_object_repeat.distance >= 0, "object::repeat::distance < 0", road_object_repeat.distance = 0);

                    road_object.repeats.push_back(road_object_repeat);
                }

                /* since v1.45 multiple <outline> are allowed and parent tag is <outlines>, not <object>; this supports v1.4 and v1.45+ */
                pugi::xml_node outlines_parent_node = object_node.child("outlines") ? object_node.child("outlines") : object_node;
                for (pugi::xml_node outline_node : outlines_parent_node.children("outline"))
                {
                    RoadObjectOutline road_object_outline(outline_node.attribute("id").as_int(-1),
                                                          outline_node.attribute("fillType").as_string(""),
                                                          outline_node.attribute("laneType").as_string(""),
                                                          outline_node.attribute("outer").as_bool(true),
                                                          outline_node.attribute("closed").as_bool(true));
                    road_object_outline.xml_node = outline_node;

                    for (pugi::xml_node corner_local_node : outline_node.children("cornerLocal"))
                    {
                        const Vec3D pt_local{corner_local_node.attribute("u").as_double(0),
                                             corner_local_node.attribute("v").as_double(0),
                                             corner_local_node.attribute("z").as_double(0)};

                        RoadObjectCorner road_object_corner_local(corner_local_node.attribute("id").as_int(-1),
                                                                  pt_local,
                                                                  corner_local_node.attribute("height").as_double(0),
                                                                  default_local_outline_type);
                        road_object_corner_local.xml_node = corner_local_node;
                        road_object_outline.outline.push_back(road_object_corner_local);
                    }

                    for (pugi::xml_node corner_road_node : outline_node.children("cornerRoad"))
                    {
                        const Vec3D pt_road{corner_road_node.attribute("s").as_double(0),
                                            corner_road_node.attribute("t").as_double(0),
                                            corner_road_node.attribute("dz").as_double(0)};

                        RoadObjectCorner road_object_corner_road(corner_road_node.attribute("id").as_int(-1),
                                                                 pt_road,
                                                                 corner_road_node.attribute("height").as_double(0),
                                                                 RoadObjectCorner::Type_Road);
                        road_object_corner_road.xml_node = corner_road_node;
                        road_object_outline.outline.push_back(road_object_corner_road);
                    }

                    road_object.outlines.push_back(road_object_outline);
                }

                road_object.lane_validities = extract_lane_validity_records(object_node);
            }
        }
        /* parse signals */
        if (with_road_signals)
        {
            for (pugi::xml_node signal_node : road_node.child("signals").children("signal"))
            {
                std::string road_signal_id = signal_node.attribute("id").as_string("");
                CHECK_AND_REPAIR(road.id_to_signal.find(road_signal_id) == road.id_to_signal.end(),
                                 (std::string("signal::id already exists - ") + road_signal_id).c_str(),
                                 road_signal_id = road_signal_id + std::string("_dup"));

                RoadSignal& road_signal = road.id_to_signal
                                              .insert({road_signal_id,
                                                       RoadSignal(road_id,
                                                                  road_signal_id,
                                                                  signal_node.attribute("name").as_string(""),
                                                                  signal_node.attribute("s").as_double(0),
                                                                  signal_node.attribute("t").as_double(0),
                                                                  signal_node.attribute("dynamic").as_bool(),
                                                                  signal_node.attribute("zOffset").as_double(0),
                                                                  signal_node.attribute("value").as_double(0),
                                                                  signal_node.attribute("height").as_double(0),
                                                                  signal_node.attribute("width").as_double(0),
                                                                  signal_node.attribute("hOffset").as_double(0),
                                                                  signal_node.attribute("pitch").as_double(0),
                                                                  signal_node.attribute("roll").as_double(0),
                                                                  signal_node.attribute("orientation").as_string("none"),
                                                                  signal_node.attribute("country").as_string(""),
                                                                  signal_node.attribute("type").as_string("none"),
                                                                  signal_node.attribute("subtype").as_string("none"),
                                                                  signal_node.attribute("unit").as_string(""),
                                                                  signal_node.attribute("text").as_string("none"))})
                                              .first->second;
                road_signal.xml_node = signal_node;

                CHECK_AND_REPAIR(road_signal.s0 >= 0, "signal::s < 0", road_signal.s0 = 0);
                CHECK_AND_REPAIR(road_signal.height >= 0, "signal::height < 0", road_signal.height = 0);
                CHECK_AND_REPAIR(road_signal.width >= 0, "signal::width < 0", road_signal.width = 0);

                road_signal.lane_validities = extract_lane_validity_records(signal_node);
            }
        }
    }

    return supported;
}

bool OpenDriveMap::Load(const std::string& xodr_file,
                        const bool         center_map,
                        const bool         with_road_objects,
                        const bool         with_lateral_profile,
                        const bool         with_lane_height,
                        const bool         abs_z_for_for_local_road_obj_outline,
                        const bool         fix_spiral_edge_cases,
                        const bool         with_road_signals)
{
    std::ifstream     ifs(xodr_file);
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    return LoadString(buffer.str(),
                      center_map,
                      with_road_objects,
                      with_lateral_profile,
                      with_lane_height,
                      abs_z_for_for_local_road_obj_outline,
                      fix_spiral_edge_cases,
                      with_road_signals);
}

std::vector<Road> OpenDriveMap::get_roads() const { return get_map_values(this->id_to_road); }

std::vector<Junction> OpenDriveMap::get_junctions() const { return get_map_values(this->id_to_junction); }

RoadNetworkMesh OpenDriveMap::get_road_network_mesh(const double eps) const
{
    RoadNetworkMesh  out_mesh;
    LanesMesh&       lanes_mesh = out_mesh.lanes_mesh;
    RoadmarksMesh&   roadmarks_mesh = out_mesh.roadmarks_mesh;
    RoadObjectsMesh& road_objects_mesh = out_mesh.road_objects_mesh;
    RoadSignalsMesh& road_signals_mesh = out_mesh.road_signals_mesh;

    for (const auto& id_road : this->id_to_road)
    {
        const Road& road = id_road.second;
        lanes_mesh.road_start_indices[lanes_mesh.vertices.size()] = road.id;
        roadmarks_mesh.road_start_indices[roadmarks_mesh.vertices.size()] = road.id;
        road_objects_mesh.road_start_indices[road_objects_mesh.vertices.size()] = road.id;

        for (const auto& s_lanesec : road.s_to_lanesection)
        {
            const LaneSection& lanesec = s_lanesec.second;
            lanes_mesh.lanesec_start_indices[lanes_mesh.vertices.size()] = lanesec.s0;
            roadmarks_mesh.lanesec_start_indices[roadmarks_mesh.vertices.size()] = lanesec.s0;
            for (const auto& id_lane : lanesec.id_to_lane)
            {
                const Lane&       lane = id_lane.second;
                const std::size_t lanes_idx_offset = lanes_mesh.vertices.size();
                lanes_mesh.lane_start_indices[lanes_idx_offset] = lane.id;
                lanes_mesh.add_mesh(road.get_lane_mesh(lane, eps));

                std::size_t roadmarks_idx_offset = roadmarks_mesh.vertices.size();
                roadmarks_mesh.lane_start_indices[roadmarks_idx_offset] = lane.id;
                const std::vector<RoadMark> roadmarks = lane.get_roadmarks(lanesec.s0, road.get_lanesection_end(lanesec));
                for (const RoadMark& roadmark : roadmarks)
                {
                    roadmarks_idx_offset = roadmarks_mesh.vertices.size();
                    roadmarks_mesh.roadmark_type_start_indices[roadmarks_idx_offset] = roadmark.type;
                    roadmarks_mesh.add_mesh(road.get_roadmark_mesh(lane, roadmark, eps));
                }
            }
        }

        for (const auto& id_road_object : road.id_to_object)
        {
            const RoadObject& road_object = id_road_object.second;
            const std::size_t road_objs_idx_offset = road_objects_mesh.vertices.size();
            road_objects_mesh.road_object_start_indices[road_objs_idx_offset] = road_object.id;
            road_objects_mesh.add_mesh(road.get_road_object_mesh(road_object, eps));
        }

        for (const auto& id_signal : road.id_to_signal)
        {
            const RoadSignal& road_signal = id_signal.second;
            const std::size_t signals_idx_offset = road_signals_mesh.vertices.size();
            road_signals_mesh.road_signal_start_indices[signals_idx_offset] = road_signal.id;
            road_signals_mesh.add_mesh(road.get_road_signal_mesh(road_signal));
        }
    }

    return out_mesh;
}

RoutingGraph OpenDriveMap::get_routing_graph() const
{
    RoutingGraph routing_graph;

    /* find lane successors/predecessors */
    // covers Common junction
    for (const bool find_successor : {true, false})
    {
        for (const auto& id_road : this->id_to_road)
        {
            const Road&     road = id_road.second;

            for (auto s_lanesec_iter = road.s_to_lanesection.begin(); s_lanesec_iter != road.s_to_lanesection.end(); s_lanesec_iter++)
            {
                const LaneSection& lanesec = s_lanesec_iter->second;
                const LaneSection* next_lanesec = nullptr;
                const Road*        next_lanesecs_road = nullptr;

                bool requireNextRoad = find_successor && std::next(s_lanesec_iter) == road.s_to_lanesection.end();
                bool requirePrevRoad = !find_successor && s_lanesec_iter == road.s_to_lanesection.begin();

                if (requireNextRoad || requirePrevRoad) {
                    const RoadLink& road_link = find_successor ? road.successor : road.predecessor;
                    if (road_link.type != RoadLink::Type_Road || road_link.contact_point == RoadLink::ContactPoint_None)
                        continue;

                    auto next_road_iter = this->id_to_road.find(road_link.id);
                    if (next_road_iter == this->id_to_road.end())
                        continue;
                    const Road&        next_road = next_road_iter->second;
                    const LaneSection& next_road_contact_lanesec = (road_link.contact_point == RoadLink::ContactPoint_Start)
                                                                       ? next_road.s_to_lanesection.begin()->second
                                                                       : next_road.s_to_lanesection.rbegin()->second;
                    if (requireNextRoad)
                    {
                        next_lanesec = &next_road_contact_lanesec; // take next road to find successor
                        next_lanesecs_road = &next_road;
                    }
                    else if (requirePrevRoad)
                    {
                        next_lanesec = &next_road_contact_lanesec; // take prev. road to find predecessor
                        next_lanesecs_road = &next_road;
                    }
                }

                else
                {
                    next_lanesec = find_successor ? &(std::next(s_lanesec_iter)->second) : &(std::prev(s_lanesec_iter)->second);
                    next_lanesecs_road = &road;
                }

                for (const auto& id_lane : lanesec.id_to_lane)
                {
                    const Lane& lane = id_lane.second;
                    const int   next_lane_id = (find_successor == lane.id < 0) ? lane.successor : lane.predecessor;
                    if (next_lane_id == 0)
                        continue;

                    auto next_lane_iter = next_lanesec->id_to_lane.find(next_lane_id);
                    if (next_lane_iter == next_lanesec->id_to_lane.end())
                        continue;
                    const Lane& next_lane = next_lane_iter->second;

                    const Lane&        from_lane = find_successor ? lane : next_lane;
                    const LaneSection& from_lanesection = find_successor ? lanesec : *next_lanesec;
                    const Road&        from_road = find_successor ? road : *next_lanesecs_road;

                    const Lane&        to_lane = find_successor ? next_lane : lane;
                    const LaneSection& to_lanesection = find_successor ? *next_lanesec : lanesec;
                    const Road&        to_road = find_successor ? *next_lanesecs_road : road;

                    const LaneKey from(from_road.id, from_lanesection.s0, from_lane.id);
                    const LaneKey to(to_road.id, to_lanesection.s0, to_lane.id);
                    const double  lane_length = road.get_lanesection_length(find_successor ? from_lanesection : to_lanesection);

                    if (id_lane.first < 0)
                    {
                        routing_graph.add_edge(RoutingGraphEdge(from, to, lane_length));
                    }
                    else
                    {
                        routing_graph.add_edge(RoutingGraphEdge(to, from, lane_length));
                    }
                }
            }
        }
    }

    /* parse direct junctions */
    for (const auto& id_junc : this->id_to_junction)
    {
        if (id_junc.second.type != odr::JunctionType::Direct)
        {
            continue;
        }
        for (const auto& id_conn : id_junc.second.id_to_connection)
        {
            const JunctionConnection& conn = id_conn.second;

            auto incoming_road_iter = this->id_to_road.find(conn.incoming_road);
            auto linked_road_iter = this->id_to_road.find(conn.connecting_road);
            if (incoming_road_iter == this->id_to_road.end() || linked_road_iter == this->id_to_road.end())
                continue;
            const Road& incoming_road = incoming_road_iter->second;
            const Road& linked_road = linked_road_iter->second;

            const bool is_succ_junc = conn.interface_provider_contact == odr::JunctionConnection::ContactPoint_End;

            const LaneSection& incoming_lanesec =
                is_succ_junc ? incoming_road.s_to_lanesection.rbegin()->second : incoming_road.s_to_lanesection.begin()->second;
            const LaneSection& linked_lanesec = (conn.contact_point == JunctionConnection::ContactPoint_Start)
                                                        ? linked_road.s_to_lanesection.begin()->second
                                                        : linked_road.s_to_lanesection.rbegin()->second;
            for (const JunctionLaneLink& lane_link : conn.lane_links)
            {
                if (lane_link.from == 0 || lane_link.to == 0)
                    continue;
                auto from_lane_iter = incoming_lanesec.id_to_lane.find(lane_link.from);
                auto to_lane_iter = linked_lanesec.id_to_lane.find(lane_link.to);
                if (from_lane_iter == incoming_lanesec.id_to_lane.end() || to_lane_iter == linked_lanesec.id_to_lane.end())
                    continue;
                const Lane& from_lane = from_lane_iter->second;
                const Lane& to_lane = to_lane_iter->second;

                const LaneKey from(incoming_road.id, incoming_lanesec.s0, from_lane.id);
                const LaneKey to(linked_road.id, linked_lanesec.s0, to_lane.id);
                const double  lane_length = incoming_road.get_lanesection_length(incoming_lanesec);
                if (is_succ_junc && from_lane.id < 0 || !is_succ_junc && from_lane.id > 0) 
                {
                    routing_graph.add_edge(RoutingGraphEdge(from, to, lane_length));
                }
                else
                {
                    routing_graph.add_edge(RoutingGraphEdge(to, from, lane_length));
                }
            }
        }
    }

    /* lane changes*/
    for (const auto& id_road : this->id_to_road)
    {
        const Road& road = id_road.second;
        for (auto s_lanesec_iter = road.s_to_lanesection.begin(); s_lanesec_iter != road.s_to_lanesection.end(); s_lanesec_iter++)
        {
            for (int side : { -1,1 })
            {
                auto parallels = s_lanesec_iter->second.get_sorted_driving_lanes(side);
                if (parallels.size() > 1)
                {
                    std::vector<LaneKey> laneKeys;
                    for (int i = 0; i != parallels.size(); ++i)
                    {
                        laneKeys.emplace_back(LaneKey(road.id, s_lanesec_iter->second.s0, parallels[i].id));
                    }
                    routing_graph.add_parallel(laneKeys);
                }
            }
        }
    }

    return routing_graph;
}

std::vector<std::tuple<LaneKey, double, LaneKey, double>> OpenDriveMap::get_routes() const
{
    std::vector<std::tuple<LaneKey, double, LaneKey, double>> rtn;

    // Find lane for each route start/end
    std::map<std::string, std::pair<LaneKey, double>> global_id_to_start, global_id_to_end;
    for (const auto& id_road : this->id_to_road)
    {
        const Road& road = id_road.second;
        for (const auto& id_object : road.id_to_object)
        {
            if (id_object.second.type == "route start" || id_object.second.type == "route end")
            {
                auto roadObj = id_object.second;

                auto objRoad = roadObj.road_id;
                auto objLaneSection = id_to_road.at(objRoad).get_lanesection(roadObj.s0);
                auto possibleLeftLanes = objLaneSection.get_sorted_driving_lanes(1);
                auto possibleLanes = objLaneSection.get_sorted_driving_lanes(-1);

                possibleLanes.insert(possibleLanes.end(), possibleLeftLanes.begin(), possibleLeftLanes.end());
                bool laneFound = false;
                for (const auto& lane : possibleLanes)
                {
                    auto innerT = lane.inner_border.get(roadObj.s0);
                    auto outerT = lane.outer_border.get(roadObj.s0);
                    if (innerT < roadObj.t0 && roadObj.t0 < outerT || outerT < roadObj.t0 && roadObj.t0 < innerT)
                    {
                        laneFound = true;
                        LaneKey laneKey(objRoad, objLaneSection.s0, lane.id);
                        std::pair<LaneKey, double> value = std::make_pair(laneKey, roadObj.s0);
                        if (id_object.second.type == "route start")
                        {
                            global_id_to_start.emplace(id_object.first, value);
                        }
                        else
                        {
                            global_id_to_end.emplace(id_object.first, value);
                        }
                        break;
                    }
                }
                if (!laneFound)
                {
                    std::cout << "[Warning] Lane of route start/end " << id_object.first << " not found\n";
                }
            }
        }
    }

    // Pair start and end
    for (const auto& id_road : this->id_to_road)
    {
        const Road& road = id_road.second;
        for (const auto& id_object : road.id_to_object)
        {
            if (id_object.second.type == "route start")
            {
                auto startInfo = global_id_to_start.find(id_object.first);
                if (startInfo == global_id_to_start.end())
                {
                    continue;
                }

                auto endID = id_object.second.name;
                auto endInfo = global_id_to_end.find(endID);
                if (endInfo == global_id_to_end.end())
                {
                    std::cout << "[Warning] Route end " << endID << " not found\n";
                    continue;
                }
                
                rtn.emplace_back(std::make_tuple(startInfo->second.first, startInfo->second.second, 
                    endInfo->second.first, endInfo->second.second));
            }
        }
    }

    return rtn;
}

std::map<LaneKey, std::vector<std::pair<LaneKey, double>>> OpenDriveMap::get_overlap_zones() const
{
    std::map<LaneKey, std::vector<std::pair<LaneKey, double>>> rtn;
    for (const auto& id_to_junction : this->id_to_junction)
    {
        if (id_to_junction.second.type == odr::JunctionType::Direct)
        {
            std::map<odr::LaneKey, std::vector<LaneKey>> incomingToLinked;
            std::map<odr::LaneKey, double>               incomingToOverlap;
            for (const auto& id_conn : id_to_junction.second.id_to_connection)
            {
                for (const auto& ll : id_conn.second.lane_links)
                {
                    if (ll.overlapZone != 0)
                    {
                        bool         merging = ((id_conn.second.contact_point == odr::JunctionConnection::ContactPoint_End) == (ll.to < 0));

                        odr::LaneKey incoming(id_conn.second.incoming_road, 0, ll.from);
                        if (merging)
                        {
                            incomingToOverlap[incoming] = std::min(-ll.overlapZone, incomingToOverlap[incoming]);
                        }
                        else
                        {
                            incomingToOverlap[incoming] = std::max(ll.overlapZone, incomingToOverlap[incoming]);
                        }
                        
                        const auto& lane_sections = id_to_road.at(id_conn.second.connecting_road).s_to_lanesection;
                        auto        lanesection_s =
                            id_conn.second.contact_point == odr::JunctionConnection::ContactPoint_Start ? 0 : lane_sections.rbegin()->first;
                        odr::LaneKey linked(id_conn.second.connecting_road, lanesection_s, ll.to);
                        incomingToLinked[incoming].push_back(linked);
                    }
                }
            }
            if (!incomingToLinked.empty())
            {
                for (const auto& incoming_to_overlap : incomingToOverlap)
                {
                    double overlep_length = incoming_to_overlap.second;
                    auto   all_overlaps = incomingToLinked.at(incoming_to_overlap.first);
                    for (const auto& overlap_linked : all_overlaps)
                    {
                        for (const auto& overlap : all_overlaps)
                        {
                            if (!std::equal_to<LaneKey>{}(overlap, overlap_linked))
                            {
                                rtn[overlap_linked].push_back(std::make_pair(overlap, overlep_length));
                            }
                        }
                        
                    }
                }
            }
        }
    }
    return rtn;
}

double OpenDriveMap::get_lanekey_length(LaneKey key) const
{
    const auto& road = id_to_road.at(key.road_id);
    const auto& section = road.get_lanesection(key.lanesection_s0);
    return road.get_lanesection_length(section);
}

void OpenDriveMap::export_file(const std::string& fpath) const
{
    pugi::xml_document doc;
    pugi::xml_node     main = doc.append_child("OpenDRIVE");
    pugi::xml_node     header = main.append_child("header");
    header.append_attribute("revMajor").set_value("1");
    header.append_attribute("revMinor").set_value("4");

    for (auto rit : id_to_road) 
    {
        const odr::Road&      road = rit.second;
        pugi::xml_node road_node = main.append_child("road");
        road_node.append_attribute("name").set_value(road.name.c_str());
        road_node.append_attribute("length").set_value(road.length);
        road_node.append_attribute("id").set_value(rit.first.c_str());
        road_node.append_attribute("junction").set_value(road.junction.c_str());

        // link
        pugi::xml_node link = road_node.append_child("link");
        if (road.predecessor.type != odr::RoadLink::Type_None) 
        {
            pugi::xml_node pred = link.append_child("predecessor");
            roadNodeToXML(road.predecessor, pred);
        }
        if (road.successor.type != odr::RoadLink::Type_None) 
        {
            pugi::xml_node succ = link.append_child("successor");
            roadNodeToXML(road.successor, succ);
        }

        // type
        for (auto it : road.s_to_type) 
        {
            pugi::xml_node type = road_node.append_child("type");
            type.append_attribute("s").set_value(it.first);
            type.append_attribute("type").set_value(it.second.c_str());
        }

        // planView
        pugi::xml_node planView = road_node.append_child("planView");
        for (auto it = road.ref_line.s0_to_geometry.begin(); it != road.ref_line.s0_to_geometry.end(); ++it) 
        {
            pugi::xml_node geometry = planView.append_child("geometry");
            geometry.append_attribute("s").set_value(it->first);
            geometry.append_attribute("x").set_value(it->second->x0);
            geometry.append_attribute("y").set_value(it->second->y0);
            geometry.append_attribute("hdg").set_value(it->second->hdg0);
            geometry.append_attribute("length").set_value(it->second->length);

            if (dynamic_cast<odr::Line*>(it->second.get()))
            {
                geometry.append_child("line");
            }
            else if (odr::Arc* geo = dynamic_cast<odr::Arc*>(it->second.get()))
            {
                geometry.append_child("arc").append_attribute("curvature").set_value(geo->curvature);
            }
            else if (odr::Spiral* geo = dynamic_cast<odr::Spiral*>(it->second.get()))
            {
                pugi::xml_node spiral = geometry.append_child("spiral");
                spiral.append_attribute("curvStart").set_value(geo->curv_start);
                spiral.append_attribute("curvEnd").set_value(geo->curv_end);
            }
            else 
            {
                odr::ParamPoly3* g = dynamic_cast<odr::ParamPoly3*>(it->second.get());
                pugi::xml_node   poly3 = geometry.append_child("paramPoly3");
                poly3.append_attribute("aU").set_value(g->aU);
                poly3.append_attribute("aV").set_value(g->aV);
                poly3.append_attribute("bU").set_value(g->bU);
                poly3.append_attribute("bV").set_value(g->bV);
                poly3.append_attribute("cU").set_value(g->cU);
                poly3.append_attribute("cV").set_value(g->cV);
                poly3.append_attribute("dU").set_value(g->dU);
                poly3.append_attribute("dV").set_value(g->dV);
                poly3.append_attribute("pRange").set_value(g->pRange_normalized ? "normalized" : "arcLength");
            }
        }

        // elevationProfile
        pugi::xml_node elevationProfile = road_node.append_child("elevationProfile");
        for (const auto& s0_poly : road.ref_line.elevation_profile.s0_to_poly) 
        {
            pugi::xml_node elevation = elevationProfile.append_child("elevation");
            elevation.append_attribute("s").set_value(s0_poly.first);
            elevation.append_attribute("a").set_value(s0_poly.second.raw_a);
            elevation.append_attribute("b").set_value(s0_poly.second.raw_b);
            elevation.append_attribute("c").set_value(s0_poly.second.raw_c);
            elevation.append_attribute("d").set_value(s0_poly.second.raw_d);
        }

        // lanes
        pugi::xml_node lanes = road_node.append_child("lanes");
        for (auto it : road.lane_offset.s0_to_poly) 
        {
            pugi::xml_node laneOffset = lanes.append_child("laneOffset");
            laneOffset.append_attribute("s").set_value(it.first);
            laneOffset.append_attribute("a").set_value(it.second.raw_a);
            laneOffset.append_attribute("b").set_value(it.second.raw_b);
            laneOffset.append_attribute("c").set_value(it.second.raw_c);
            laneOffset.append_attribute("d").set_value(it.second.raw_d);
        }
        for (auto sectionIt : road.s_to_lanesection) 
        {
            pugi::xml_node laneSection = lanes.append_child("laneSection");
            laneSection.append_attribute("s").set_value(sectionIt.first);
            bool leftCreated = false, rightCreated = false;

            pugi::xml_node left, right;

            for (auto laneIt : sectionIt.second.id_to_lane) 
            {
                pugi::xml_node lane;
                odr::Lane      l = laneIt.second;
                if (laneIt.first > 0) 
                {   
                    if (!leftCreated) 
                    {
                        left = laneSection.append_child("left");
                        leftCreated = true;
                    }
                    lane = left.append_child("lane");
                }
                else if (laneIt.first == 0)
                {
                    pugi::xml_node center = laneSection.append_child("center");
                    lane = center.append_child("lane");
                }
                else
                {
                    if (!rightCreated) 
                    {
                        right = laneSection.append_child("right");
                        rightCreated = true;
                    }
                    lane = right.append_child("lane");
                }
                lane.append_attribute("id").set_value(laneIt.first);
                lane.append_attribute("type").set_value(l.type.c_str());
                lane.append_attribute("level").set_value(l.level ? "true" : "false");
                if (l.predecessor != 0 || l.successor != 0) 
                {
                    pugi::xml_node link = lane.append_child("link");
                    if (l.predecessor != 0)
                    {
                        link.append_child("predecessor").append_attribute("id").set_value(l.predecessor);
                    }
                    if (l.successor != 0)
                    {
                        link.append_child("successor").append_attribute("id").set_value(l.successor);
                    }
                }
                for (auto laneWidthIt : l.lane_width.s0_to_poly) 
                {
                    pugi::xml_node width = lane.append_child("width");
                    width.append_attribute("sOffset").set_value(laneWidthIt.first - sectionIt.first);
                    width.append_attribute("a").set_value(laneWidthIt.second.raw_a);
                    width.append_attribute("b").set_value(laneWidthIt.second.raw_b);
                    width.append_attribute("c").set_value(laneWidthIt.second.raw_c);
                    width.append_attribute("d").set_value(laneWidthIt.second.raw_d);
                }
                // Not sorted
                for (auto roadMarkIt : l.roadmark_groups) 
                {
                    pugi::xml_node roadMark = lane.append_child("roadMark");
                    roadMark.append_attribute("sOffset").set_value(roadMarkIt.s_offset);
                    roadMark.append_attribute("type").set_value(roadMarkIt.type.c_str());
                    roadMark.append_attribute("color").set_value(roadMarkIt.color.c_str());
                    if (roadMarkIt.height != 0) 
                        roadMark.append_attribute("height").set_value(roadMarkIt.height);
                    if (roadMarkIt.lane_change != "both")
                        roadMark.append_attribute("laneChange").set_value(roadMarkIt.lane_change.c_str());
                    if (roadMarkIt.material != "standard")
                        roadMark.append_attribute("material").set_value(roadMarkIt.material.c_str());
                    if (roadMarkIt.weight != "standard")
                        roadMark.append_attribute("weight").set_value(roadMarkIt.weight.c_str());
                    if (roadMarkIt.width >= 0)
                        roadMark.append_attribute("width").set_value(roadMarkIt.width);

                    if (!roadMarkIt.roadmark_lines.empty()) 
                    {
                        auto typeChild = roadMark.append_child("type");
                        for (auto roadMarkLineIt : roadMarkIt.roadmark_lines)
                        {
                            pugi::xml_node roadMarkLine = typeChild.append_child("line");
                            roadMarkLine.append_attribute("length").set_value(roadMarkLineIt.length);
                            roadMarkLine.append_attribute("space").set_value(roadMarkLineIt.space);
                            roadMarkLine.append_attribute("width").set_value(roadMarkLineIt.width);
                            roadMarkLine.append_attribute("sOffset").set_value(roadMarkLineIt.s_offset);
                            roadMarkLine.append_attribute("tOffset").set_value(roadMarkLineIt.t_offset);
                            if (!roadMarkLineIt.rule.empty() != 0)
                                roadMarkLine.append_attribute("rule").set_value(roadMarkLineIt.rule.c_str());
                        }
                    }
                }
                // Missing user data
            }
        }

        // road objects
        if (!road.id_to_object.empty())
        {
            auto objects_node = road_node.append_child("objects");
            for (const auto& id_object : road.id_to_object)
            {
                auto road_object = id_object.second;
                auto object_node = objects_node.append_child("object");
                object_node.append_attribute("id").set_value(id_object.first.c_str());
                object_node.append_attribute("dynamic").set_value(road_object.is_dynamic);
                object_node.append_attribute("s").set_value(road_object.s0);
                object_node.append_attribute("t").set_value(road_object.t0);
                object_node.append_attribute("zOffset").set_value(road_object.z0);
                object_node.append_attribute("length").set_value(road_object.length);
                object_node.append_attribute("validLength").set_value(road_object.valid_length);
                object_node.append_attribute("width").set_value(road_object.width);
                object_node.append_attribute("radius").set_value(road_object.radius);
                object_node.append_attribute("height").set_value(road_object.height);
                object_node.append_attribute("hdg").set_value(road_object.hdg);
                object_node.append_attribute("pitch").set_value(road_object.pitch);
                object_node.append_attribute("roll").set_value(road_object.roll);
                object_node.append_attribute("type").set_value(road_object.type.c_str());
                object_node.append_attribute("name").set_value(road_object.name.c_str());
                object_node.append_attribute("orientation").set_value(road_object.hdg);
                object_node.append_attribute("subtype").set_value(road_object.subtype.c_str());
                // currently no repeat or outline child
            }
        }
        
        pugi::xml_node customProfile = road_node.append_child("roadRunnerProfile");
        if (!road.rr_profile.leftPlans.empty()) 
        {
            pugi::xml_node customLeft = customProfile.append_child("left");
            for (auto customProfile : road.rr_profile.leftPlans)
            {
                pugi::xml_node section = customLeft.append_child("section");
                section.append_attribute("type_s").set_value(customProfile.first);
                section.append_attribute("laneCount").set_value(customProfile.second.laneCount);
                section.append_attribute("offsetX2").set_value(customProfile.second.offsetx2);
            }
        }
        if (!road.rr_profile.rightPlan.empty()) 
        {
            pugi::xml_node customRight = customProfile.append_child("right");
            for (auto customProfile : road.rr_profile.rightPlan) 
            {
                pugi::xml_node section = customRight.append_child("section");
                section.append_attribute("type_s").set_value(customProfile.first);
                section.append_attribute("laneCount").set_value(customProfile.second.laneCount);
                section.append_attribute("offsetX2").set_value(customProfile.second.offsetx2);
            }
        }

        if (!road.boundaryHide.empty())
        {
            pugi::xml_node customBoundaryHide = road_node.append_child("roadRunnerBoundaryHide");
            for (auto boundary_length : road.boundaryHide)
            {
                auto           boundary = boundary_length.first;
                pugi::xml_node hideDetail = customBoundaryHide.append_child("hide");
                hideDetail.append_attribute("contactPoint").set_value(
                    boundary.first == odr::RoadLink::ContactPoint_Start ? "start" : "end");
                hideDetail.append_attribute("side").set_value(boundary.second == -1 ? "right" : "left");
                hideDetail.append_attribute("s").set_value(boundary_length.second);
            }
        }
    }

    for (auto j : get_junctions()) 
    {
        pugi::xml_node junction = main.append_child("junction");
        junction.append_attribute("id").set_value(j.id.c_str());
        junction.append_attribute("name").set_value(j.name.c_str());
        if (j.type == JunctionType::Direct) 
        {
            junction.append_attribute("type").set_value("direct");   
        }
        for (auto c : j.id_to_connection) 
        {
            auto connection = junction.append_child("connection");
            connection.append_attribute("id").set_value(c.first.c_str());

            std::string incomingRoad = c.second.incoming_road;
            if (incomingRoad.size() > 0)
                connection.append_attribute("incomingRoad").set_value(incomingRoad.c_str());
            connection.append_attribute(j.type == JunctionType::Common ? "connectingRoad" : "linkedRoad").set_value(c.second.connecting_road.c_str());

            std::string contectPoint;
            switch (c.second.contact_point)
            {
            case odr::JunctionConnection::ContactPoint_Start:
                contectPoint = "start";
                break;
            case odr::JunctionConnection::ContactPoint_End:
                contectPoint = "end";
                break;
            default:
                break;
            }
            if (!contectPoint.empty())
                connection.append_attribute("contactPoint").set_value(contectPoint.c_str());

            std::string interfaceProviderContect;
            switch (c.second.interface_provider_contact)
            {
            case odr::JunctionConnection::ContactPoint_Start:
                interfaceProviderContect = "start";
                break;
            case odr::JunctionConnection::ContactPoint_End:
                interfaceProviderContect = "end";
                break;
            default:
                break;
            }
            if (!interfaceProviderContect.empty())
                connection.append_attribute("interfaceProviderContactPoint").set_value(interfaceProviderContect.c_str());

            for (auto p: c.second.signalPhases)
            {
                connection.append_child("signalPhase").append_attribute("id").set_value(p);
            }

            for (auto ll : c.second.lane_links) 
            {
                pugi::xml_node laneLink = connection.append_child("laneLink");
                laneLink.append_attribute("from").set_value(ll.from);
                laneLink.append_attribute("to").set_value(ll.to);
                if (ll.overlapZone > 0)
                {
                    laneLink.append_attribute("overlapZone").set_value(ll.overlapZone);
                }
            }
        }

        // Modified version based on xodr
        auto boundary_node = junction.append_child("boundary");
        for (auto segment : j.boundary)
        {
            auto segment_node = boundary_node.append_child("segment");
            segment_node.append_attribute("roadID").set_value(segment.road.c_str());
            segment_node.append_attribute("sStart").set_value(segment.sBegin);
            segment_node.append_attribute("sEnd").set_value(segment.sEnd);
            segment_node.append_attribute("side").set_value(segment.side > 0 ? "left" : "right");
            segment_node.append_attribute("type").set_value(segment.type == odr::BoundarySegmentType::Lane ? "lane" : "joint");
        }
    }
    // junctions
    doc.save_file(fpath.c_str());
}

void OpenDriveMap::roadNodeToXML(const odr::RoadLink& roadLink, pugi::xml_node& out) const 
{ 
    std::string type_s;
    switch (roadLink.type)
    {
    case odr::RoadLink::Type_Road:
        type_s = "road";
        break;
    case odr::RoadLink::Type_Junction:
        type_s = "junction";
        break;
    default:
        break;
    }
    out.append_attribute("elementType").set_value(type_s.c_str());
    out.append_attribute("elementId").set_value(roadLink.id.c_str());
    std::string contact_s;
    switch (roadLink.contact_point)
    {
    case odr::RoadLink::ContactPoint::ContactPoint_Start:
        contact_s = "start";
        break;
    case odr::RoadLink::ContactPoint::ContactPoint_End:
        contact_s = "end";
        break;
    default:
        break;
    }
    out.append_attribute("contactPoint").set_value(contact_s.c_str());
}
}
