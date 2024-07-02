#pragma once

#include "OpenDriveMap.h"
#include "road.h"
#include <map>
#include <vector>
#include <functional>

namespace RoadRunnerTest { class Validation; }

namespace RoadRunner
{
    // public
    class ConnectionInfo
    {
    public:
        ConnectionInfo() = delete;

        ConnectionInfo(std::shared_ptr<Road> inRoad, odr::RoadLink::ContactPoint ct, uint8_t skipInner=0) :
            road(inRoad), contact(ct), skipProviderLanes(skipInner)
        {
            if (inRoad != nullptr)
            {
                bool contactAtStart = contact == odr::RoadLink::ContactPoint_Start;
                leftProfile = contactAtStart ? inRoad->generated.rr_profile.LeftExit() : inRoad->generated.rr_profile.LeftEntrance();
                rightProfile = contactAtStart ? inRoad->generated.rr_profile.RightEntrance() : inRoad->generated.rr_profile.RightExit();
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

        bool operator<(const ConnectionInfo& rhs) const
        {
            if (road.expired()) return true;
            if (rhs.road.expired()) return false;
            return road.lock()->ID() < rhs.road.lock()->ID() || 
                road.lock()->ID() == rhs.road.lock()->ID() && contact < rhs.contact;
        }

        std::weak_ptr<Road> road;
        odr::RoadLink::ContactPoint contact;
        std::vector<double> dirSplit; // not needed for outgoing-only connection

        uint8_t skipProviderLanes; // Skip number of innermost linked lanes of InterfaceProvider

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
        std::vector<std::shared_ptr<Road>>& connectings);

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
            return other.road->id == road->id && other.contact == contact;
        }

        bool operator <(const RoadEndpoint& rhs) const
        {
            return road->id < rhs.road->id || road->id == rhs.road->id && contact < rhs.contact;
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
    // Only used to unlink the last (only) road when all rest
    void clearLinkage(std::string junctionID, std::string regularRoad);

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

    class AbstractJunction : public std::enable_shared_from_this<AbstractJunction>
    {
        friend class RoadRunnerTest::Validation;
    public:
        AbstractJunction();

        AbstractJunction(const odr::Junction& serialized);

        ~AbstractJunction();

        virtual int CreateFrom(const std::vector<ConnectionInfo>&) = 0;

        int Attach(ConnectionInfo);

        void NotifyPotentialChange(); /*Call this when connected roads get deleted or modified*/
        void NotifyPotentialChange(const ChangeInConnecting&);

        virtual void AttachNoRegenerate(ConnectionInfo);
        void DetachNoRegenerate(std::shared_ptr<Road>);

        std::string ID() const { return generated.id; }

        std::set< std::shared_ptr<Road>> StillConnectedRoads() const;

        void FillConnectionInfo(ConnectionInfo&) const;

        virtual std::string Log() const;

        odr::Junction generated;

        int generationError = 0;
#ifndef G_TEST
    protected:
#endif
        std::set<ConnectionInfo> formedFrom;
    };

    // TODO: inherit same class as Road to manage ID
    class Junction: public AbstractJunction
    {
    public:
        Junction();

        Junction(const odr::Junction& serialized);

        Junction(const Junction& another) = delete; // No copy costruct
        Junction& operator=(const Junction& another) = delete; // No copy assignment

        virtual int CreateFrom(const std::vector<ConnectionInfo>&) override;

        std::vector<std::shared_ptr<Road>> connectingRoads;
    };

    class DirectJunction : public AbstractJunction
    {
    public:
        DirectJunction(ConnectionInfo aInterfaceProvider);

        DirectJunction(const odr::Junction& serialized);

        virtual int CreateFrom(const std::vector<ConnectionInfo>&) override;

        virtual void AttachNoRegenerate(ConnectionInfo) override;

        virtual std::string Log() const override;

    private:
        ConnectionInfo InterfaceProvider() const;

        odr::Vec2D interfaceDir; // Vector pointing into the interface provider

        static odr::Vec2D calcInterfaceDir(const ConnectionInfo&);
    };
}
