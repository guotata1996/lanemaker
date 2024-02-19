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
        Road* road;
        double s;
        std::vector<double> dirSplit; // not needed for outgoing-only connection
    };

    enum JunctionError
    {
        Junction_TooManyIncomingLanes = 1,
        Junction_ConnectionInvalidShape = 2,
        Junction_AlgoFail = 4
    };

    int GenerateConnections(
        std::string junctionID,
        std::vector<ConnectionInfo> connected,
        std::vector<Road>& connectings);

    // internal
    // ALL lanes of a single direction, going into or coming from a junction
    struct RoadEndpoint
    {
        odr::Road* road;
        odr::RoadLink::ContactPoint contact;
        int8_t side; //-1: right, +1: left
        odr::Vec2D origin;
        odr::Vec2D forward; // cross-section = origin ~ origin + rot_right(hdg) * nLanes * LaneWidth
        uint8_t nLanes;

        std::vector<double> dirSplit; // Left empty for auto assignment.
    };

    enum TurningSemantics {Turn_No, Turn_U, Turn_Left, Turn_Right};

    // A group of neighboring lanes going from road A to B
    struct TurningGroup
    {
        odr::Vec2D fromOrigin, fromForward;
        odr::Vec2D toOrigin, toForward;
        odr::RoadLink::ContactPoint fromContact, toContact;
        int8_t fromSide, toSide;

        TurningSemantics direction;
        uint8_t toTotalLanes;

        // e.g. For the first & second driving lanes (counted from center), base = 0
        //      For the second driving lane, base = 1
        uint8_t fromLaneIDBase, toLaneIDBase;
        uint8_t nLanes;
    };

    // @param: from center to right lanes
    std::vector<double> assignIncomingLanes(int8_t nLanes, const std::vector<TurningGroup>& outgoings, int& errorCode);

    std::vector<std::pair<uint8_t, uint8_t>> splitPointToLaneCount(int8_t nLanes, std::vector<double> splitPoints);

    // @param: from center to right lanes
    // @returns: Span = [rtn, rtn+lane_count-1]
    void assignOutgoingLanes(std::vector<TurningGroup>& incomingLanes);

    // Clear any connection of regularRoad towards junction
    void clearLinkage(std::string junctionID, std::string regularRoad);

    // TODO: inherit same class as Road to manage ID
    class Junction
    {
    public:
        Junction(const std::vector<ConnectionInfo>&, std::string id="");

        Junction(const Junction& another) = delete; // No copy costruct
        Junction& operator=(const Junction& another) = delete; // No copy assignment

        // Move constructor
        Junction(Junction&& other) noexcept :
            generated(other.generated)
        {
            IDGenerator::ForJunction()->FreeID(other.ID());
            other.generated.id = "";
            generated.id = IDGenerator::ForJunction()->GenerateID(this);

            for (int i = 0; i != other.connectingRoads.size(); ++i)
            {
                connectingRoads.push_back(Road(std::move(other.connectingRoads[i])));
            }
        }

        ~Junction(); // TODO: Disconnect lane linkage, destroy connecting roads

        std::string ID() { return generated.id; }

        std::vector<Road> connectingRoads;

        odr::Junction generated;

        int generationError = 0;
    };
}
