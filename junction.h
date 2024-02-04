#pragma once

#include "OpenDriveMap.h"
#include "road.h"
#include <map>
#include <vector>

namespace RoadRunner
{
    struct ConnectionInfo
    {
        Road config;
        odr::Road* gen;
        double s;
        std::vector<double> dirSplit; // not needed for outgoing-only connection
    };

    odr::Junction GenerateConnections(
        std::vector<ConnectionInfo> connected,
        std::vector<odr::Road>& connectings);

    struct RoadEndpoint
    {
        // A set of neighboring lanes going from road A OR to road A
        std::string roadID;
        odr::Vec2D origin;
        odr::Vec2D forward; // cross-section = origin ~ origin + rot_right(hdg) * nLanes * LaneWidth
        int8_t nLanes;

        std::vector<double> dirSplit;
    };
}
