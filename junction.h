#pragma once

#include "OpenDriveMap.h"
#include "road.h"
#include <map>
#include <vector>
#include <functional>

namespace RoadRunner
{
    // public
    class ConnectionInfo
    {
    public:
        ConnectionInfo() = delete;

        ConnectionInfo(std::shared_ptr<Road> inRoad, odr::RoadLink::ContactPoint ct) :
            road(inRoad), contact(ct) 
        {
            if (inRoad != nullptr)
            {
                bool contactAtStart = contact == odr::RoadLink::ContactPoint_Start;
                leftProfile = contactAtStart ? inRoad->profile.LeftExit() : inRoad->profile.LeftEntrance();
                rightProfile = contactAtStart ? inRoad->profile.RightEntrance() : inRoad->profile.RightExit();
                refLinePos = inRoad->RefLine().get_xy(contactAtStart ? 0 : inRoad->Length());
                refLineHdg = inRoad->RefLine().get_hdg(contactAtStart ? 0 : inRoad->Length());
            }
        }

        bool operator==(const ConnectionInfo& other) const
        {
            return road.lock() == other.road.lock() &&
                contact == other.contact &&
                leftProfile == other.leftProfile &&
                rightProfile == other.rightProfile &&
                refLinePos == other.refLinePos &&
                refLineHdg == other.refLineHdg;
        }

        bool operator!=(const ConnectionInfo& other) const
        {
            return !(*this == other);
        }

        struct Hasher
        {
            size_t operator()(const ConnectionInfo& obj) const {
                return std::hash<double>()(obj.refLinePos[0]) ^ std::hash<double>()(obj.refLinePos[1]) ^ std::hash<double>()(obj.refLineHdg) ^
                    std::hash<type_t>()(obj.leftProfile.laneCount) ^ std::hash<type_t>()(obj.leftProfile.offsetx2) ^
                    std::hash<type_t>()(obj.rightProfile.laneCount) ^ std::hash<type_t>()(obj.rightProfile.offsetx2) ^
                    std::hash<odr::RoadLink::ContactPoint>()(obj.contact);
            };
        };

        std::weak_ptr<Road> road;
        odr::RoadLink::ContactPoint contact;
        std::vector<double> dirSplit; // not needed for outgoing-only connection

    private:
        SectionProfile leftProfile;
        SectionProfile rightProfile;
        odr::Vec2D refLinePos;
        double refLineHdg;
    };

    enum JunctionError
    {
        Junction_TooManyIncomingLanes = 1,
        Junction_ConnectionInvalidShape = 2,
        Junction_AlgoFail = 4,
        Junction_DuplicateConn = 8
    };

    int GenerateConnections(
        std::string junctionID,
        std::vector<ConnectionInfo> connected,
        std::vector<std::unique_ptr<Road>>& connectings);

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

        bool operator==(const RoadEndpoint& other) const
        {
            return other.road == road && other.contact == contact;
        }

        bool operator <(const RoadEndpoint& rhs) const
        {
            return road < rhs.road || road == rhs.road && contact < rhs.contact;
        }
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
    // void clearLinkage(std::string junctionID, std::string regularRoad);

    struct ChangeInConnecting
    {
        enum Type {
            Type_Others,
            Type_Reverse,
            Type_DetachAtEnd_Temp // Intermediate step; Doesn't trigger a regeneration
        };
        std::weak_ptr<Road> subject;
        Type _type;
    };

    // TODO: inherit same class as Road to manage ID
    class Junction: public std::enable_shared_from_this<Junction>
    {
    public:
        Junction(std::string id="");

        Junction(const Junction& another) = delete; // No copy costruct
        Junction& operator=(const Junction& another) = delete; // No copy assignment

        ~Junction();

        int CreateFrom(const std::vector<ConnectionInfo>&);
        int Attach(ConnectionInfo);

        /*Call this when connected roads get deleted or modified*/
        void NotifyPotentialChange();
        void NotifyPotentialChange(const ChangeInConnecting&);

        std::string ID() { return generated.id; }

        std::vector<std::unique_ptr<Road>> connectingRoads;

        odr::Junction generated;

        int generationError = 0;
#ifndef G_TEST
    private:
#endif
        std::unordered_set<ConnectionInfo, ConnectionInfo::Hasher> formedFrom;
    };
}
