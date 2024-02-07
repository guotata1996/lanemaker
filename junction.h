#pragma once

#include "OpenDriveMap.h"
#include "road.h"
#include <map>
#include <vector>

namespace RoadRunner
{
    // public
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

    // internal
    // ALL lanes of a single direction, going into or coming from a junction
    struct RoadEndpoint
    {
        std::string roadID;
        odr::Vec2D origin;
        odr::Vec2D forward; // cross-section = origin ~ origin + rot_right(hdg) * nLanes * LaneWidth
        int8_t nLanes;

        std::vector<double> dirSplit; // Left empty for auto assignment.
    };

    enum TurningSemantics {No, U, Left, Right};

    // A group of neighboring lanes going from road A to B
    struct TurningGroup
    {
        odr::Vec2D fromOrigin, fromForward;
        odr::Vec2D toOrigin, toForward;

        TurningSemantics direction;
        int8_t toTotalLanes;

        int8_t fromLaneIDBase, toLaneIDBase; // e.g. For lane {-2,-3}, base = -2
        int8_t nLanes;
    };

    // @param: from center to right lanes
    std::vector<double> assignIncomingLanes(int8_t nLanes, const std::vector<TurningGroup>& outgoings);

    std::vector<std::pair<int, int>> splitPointToLaneCount(int8_t nLanes, std::vector<double> splitPoints);

    // @param: from center to right lanes
    // @returns: Span = [rtn, rtn+lane_count-1]
    void assignOutgoingLanes(std::vector<TurningGroup>& incomingLanes);
}
