#pragma once

#include "OpenDriveMap.h"
#include "road.h"

#include <map>
#include <vector>
#include <functional>
#include <optional>

namespace RoadRunnerTest { class Validation; }

namespace RoadRunner
{
    class ChangeTracker;
    class JunctionGraphics;

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
        LanePlan leftProfile;
        LanePlan rightProfile;
        odr::Vec2D refLinePos;
        double refLineHdg;
    };

    enum JunctionError
    {
        Junction_NoError = 0,
        Junction_NoIncomingLanes = 1,
        Junction_TooManyIncomingLanes = 2,
        Junction_ConnectionInvalidShape = 4,
        Junction_AlgoFail = 8,
        Junction_DuplicateConn = 16,
        Junction_DirectNoProvider = 32
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

        bool operator==(const RoadEndpoint& other) const
        {
            return other.road->id == road->id && other.contact == contact;
        }

        bool operator <(const RoadEndpoint& rhs) const
        {
            return road->id < rhs.road->id || road->id == rhs.road->id && contact < rhs.contact;
        }

        static void FromConnInfo(const ConnectionInfo&, RoadEndpoint& incoming, RoadEndpoint& outgoing);
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
        friend class ChangeTracker;
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

        virtual void CheckForDegeneration() = 0; /*If only 2 roads left, check if they can be joined*/

        virtual void GenerateGraphics() = 0;

        std::string ID() const { return generated.id; }

        void FillConnectionInfo(ConnectionInfo&) const;

        double Elevation() const;

        virtual std::string Log() const;

        int generationError = 0;
#ifndef G_TEST
    protected:
#endif
        std::set<ConnectionInfo> formedFrom;

        odr::Junction generated;

    protected:
#ifndef G_TEST
        std::unique_ptr<JunctionGraphics> junctionGraphics;
#endif
    };

    // TODO: inherit same class as Road to manage ID
    class Junction: public AbstractJunction
    {
        friend class RoadRunnerTest::Validation;
        friend class ChangeTracker;
    public:
        Junction();

        Junction(const odr::Junction& serialized);

        Junction(const Junction& another) = delete; // No copy costruct
        Junction& operator=(const Junction& another) = delete; // No copy assignment

        virtual int CreateFrom(const std::vector<ConnectionInfo>&) override;

        virtual void CheckForDegeneration() override;

        virtual void GenerateGraphics() override;

    protected:
        std::vector<std::shared_ptr<Road>> connectingRoads;

    private:
        odr::Line2D CalcBoundary() const;
    };

    class DirectJunction : public AbstractJunction
    {
    public:
        DirectJunction(ConnectionInfo aInterfaceProvider);

        DirectJunction(const odr::Junction& serialized);

        virtual int CreateFrom(const std::vector<ConnectionInfo>&) override;

        virtual void AttachNoRegenerate(ConnectionInfo) override;

        virtual void CheckForDegeneration() override;
#ifndef G_TEST
        virtual void GenerateGraphics() override;
#endif
        virtual std::string Log() const override;

    private:
        std::optional<ConnectionInfo> InterfaceProvider() const;

        std::vector<odr::Vec2D> CalcCavity() const;

        odr::Vec2D interfaceDir; // Vector pointing into the interface provider

        static odr::Vec2D calcInterfaceDir(const ConnectionInfo&);

        /*right side: relative to interface provider
        * outS1 / outS2 is relative to zero-point at interface. If no overlap, return road length.
        * Return: true if intersection found
        */
        static bool bordersIntersect(odr::RoadLink::ContactPoint interfaceProviderContact,
            ConnectionInfo infoA, int sideA,
            ConnectionInfo infoB, int sideB,
            double& outS1, double& outS2, odr::Vec2D& outPos);
    };
}
