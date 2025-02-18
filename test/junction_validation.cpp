#include "validation.h"

#include "junction.h"

#include "test_macros.h"
#include "constants.h"

#ifdef G_TEST
#include <gtest/gtest.h>
#endif

namespace LTest
{
    void Validation::VerifyJunction(LM::AbstractJunction* junction)
    {
#ifdef G_TEST
        EXPECT_EQ(junction->generationError, 0);
#else
        if (junction->generationError != LM::Junction_NoError)
        {
            throw std::logic_error("Junction must be free of generation error!");
            return;
    }
#endif

        // Non-dummy linkage info
        ExpectGTOrAssert(junction->generated.id_to_connection.size(), 0);
        for (auto id_conn : junction->generated.id_to_connection)
        {
            ExpectGTOrAssert(id_conn.second.lane_links.size(), 0);
        }

        auto commonJunction = dynamic_cast<LM::Junction*>(junction);
        if (commonJunction != nullptr)
        {
            VerifyCommonJunction(commonJunction);
        }
        auto directJunction = dynamic_cast<LM::DirectJunction*>(junction);
        if (directJunction != nullptr)
        {
            VerifyDirectJunction(directJunction);
        }
    }

    void Validation::VerifyCommonJunction(const LM::Junction* junction)
    {
        // Make sure Road<->Junction pointers match what's declared on xodr
        std::set<std::string> connectingIDFromOdr, connectingIDFromRR;
        for (auto odrLink : junction->generated.id_to_connection)
        {
            ExpectOrAssert(odrLink.second.contact_point != odr::JunctionConnection::ContactPoint_None);
            ExpectOrAssert(odrLink.second.interface_provider_contact == odr::JunctionConnection::ContactPoint_None);

            connectingIDFromOdr.insert(odrLink.second.connecting_road);
            auto incomingRRRoad = IDGenerator::ForType(IDType::Road)->GetByID<LM::Road>(odrLink.second.incoming_road);
            bool incomingContactEnd = incomingRRRoad->successorJunction == junction->shared_from_this();
            bool incomingContactStart = incomingRRRoad->predecessorJunction == junction->shared_from_this();
            ExpectOrAssert(incomingContactEnd || incomingContactStart);

            if (incomingContactEnd)
            {
                LM::ConnectionInfo info(incomingRRRoad->shared_from_this(), odr::RoadLink::ContactPoint_End);
                ExpectOrAssert(junction->formedFrom.find(info) != junction->formedFrom.end());
            }
            if (incomingContactStart)
            {
                LM::ConnectionInfo info(incomingRRRoad->shared_from_this(), odr::RoadLink::ContactPoint_Start);
                ExpectOrAssert(junction->formedFrom.find(info) != junction->formedFrom.end());
            }

            auto connectingRRRoad = IDGenerator::ForType(IDType::Road)->GetByID<LM::Road>(odrLink.second.connecting_road);
            auto outgoingRoadID = connectingRRRoad->generated.predecessor.id;
            auto outgoingRRRoad = IDGenerator::ForType(IDType::Road)->GetByID<LM::Road>(outgoingRoadID);
            bool outgoingContactEnd = outgoingRRRoad->successorJunction == junction->shared_from_this();
            bool outgoingContactStart = outgoingRRRoad->predecessorJunction == junction->shared_from_this();
            ExpectOrAssert(outgoingContactEnd || outgoingContactStart);

            if (outgoingContactEnd)
            {
                LM::ConnectionInfo info(outgoingRRRoad->shared_from_this(), odr::RoadLink::ContactPoint_End);
                ExpectOrAssert(junction->formedFrom.find(info) != junction->formedFrom.end());
            }
            if (outgoingContactStart)
            {
                LM::ConnectionInfo info(outgoingRRRoad->shared_from_this(), odr::RoadLink::ContactPoint_Start);
                ExpectOrAssert(junction->formedFrom.find(info) != junction->formedFrom.end());
            }
        }
        for (auto road : junction->connectingRoads)
        {
            connectingIDFromRR.insert(road->ID());
        }

        ExpectOrAssert(connectingIDFromOdr == connectingIDFromRR);

        // Make sure all incoming roads' entering lanes have matching connectings
        auto allConnections = odr::get_map_values(junction->generated.id_to_connection);
        for (auto& incomingInfo : junction->formedFrom)
        {
            const odr::Road& incomingRoad = incomingInfo.road.lock()->generated;
            auto link = incomingInfo.contact == odr::RoadLink::ContactPoint_Start ? incomingRoad.predecessor : incomingRoad.successor;

            ExpectOrAssert(link.type == odr::RoadLink::Type_Junction);
            ExpectOrAssert(link.id == junction->generated.id);

            std::map<int, std::pair<std::string, int>> incomingLaneToConnectingRoadLane;
            for (const auto& connection : allConnections)
            {
                if (connection.incoming_road == incomingRoad.id)
                {
                    ExpectOrAssert(connection.contact_point == odr::JunctionConnection::ContactPoint_Start);
                    for (const auto& ll : connection.lane_links)
                    {
                        incomingLaneToConnectingRoadLane.emplace(ll.from, std::make_pair(connection.connecting_road, ll.to));
                    }
                }
            }

            auto enteringSection = incomingInfo.contact == odr::RoadLink::ContactPoint_Start ?
                incomingRoad.s_to_lanesection.begin()->second : incomingRoad.s_to_lanesection.rbegin()->second;
            auto enteringLanes = enteringSection.get_sorted_driving_lanes(
                incomingInfo.contact == odr::RoadLink::ContactPoint_Start ? 1 : -1);
            for (const odr::Lane& enteringLane : enteringLanes)
            {
                auto connectingRoadAndLane = incomingLaneToConnectingRoadLane.at(enteringLane.id);
                std::string connectingRoadID = connectingRoadAndLane.first;
                int connectingLane = connectingRoadAndLane.second;
                auto connectingRoad = std::find_if(junction->connectingRoads.begin(), junction->connectingRoads.end(),
                    [connectingRoadID](auto& road) {return road->ID() == connectingRoadID; });

                double incomingS = incomingInfo.contact == odr::RoadLink::ContactPoint_Start ? 0 : incomingRoad.length;
                EnsureEndsMeet(&incomingRoad, incomingS, enteringLane.id,
                    &(*connectingRoad)->generated, 0, connectingLane);
            }
        }

        // Make sure all connecting roads have matching outgoing lanes
        for (auto& connecting : junction->connectingRoads)
        {
            auto outLink = connecting->generated.successor;

            ExpectOrAssert(connecting->generated.junction == junction->generated.id);
            ExpectOrAssert(outLink.type == odr::RoadLink::Type_Road);

            std::string outgoingID = outLink.id;
            std::string connectingID = connecting->ID();
            auto connection = std::find_if(allConnections.begin(), allConnections.end(),
                [connectingID](auto& connection) {return connectingID == connection.connecting_road; });

            auto contactPointOnNext = outLink.contact_point;
            auto outgoingItr = std::find_if(junction->formedFrom.begin(), junction->formedFrom.end(),
                [outgoingID, contactPointOnNext](const LM::ConnectionInfo& info)
                {
                    return info.road.lock()->ID() == outgoingID && info.contact == contactPointOnNext;
                });
            for (const auto& ll : connection->lane_links)
            {
                double outgoingS = outgoingItr->contact == odr::RoadLink::ContactPoint_Start ? 0 : outgoingItr->road.lock()->Length();
                int outgoingLane = connecting->generated.s_to_lanesection.rbegin()->second.id_to_lane.at(ll.to).successor;
                EnsureEndsMeet(&connecting->generated, connecting->generated.length, ll.to,
                    &outgoingItr->road.lock()->generated, outgoingS, outgoingLane);
            }
        }
    }

    void Validation::EnsureEndsMeet(const odr::Road* road1, double s1, odr::Lane l1,
        const odr::Road* road2, double s2, odr::Lane l2)
    {
        double t1_out = l1.outer_border.get(s1);
        double t2_out = l2.outer_border.get(s2);
        odr::Vec3D p1_out = road1->get_xyz(s1, t1_out, 0);  // Note: This returns global pos. reflane.get_xy returns local pos.
        odr::Vec3D p2_out = road2->get_xyz(s2, t2_out, 0);
#ifdef G_TEST
        EXPECT_LT(odr::euclDistance(p1_out, p2_out), LM::epsilon)
            << road1->id << " " << l1.id << " outer border doesn't meet " << road2->id << " " << l2.id
            << "(" << p1_out[0] << " , " << p1_out[1] << ") vs (" << p2_out[0] << " , " << p2_out[1] << ")";
#else
        ExpectOrAssert(odr::euclDistance(p1_out, p2_out) < LM::epsilon);
#endif

        double t1_in = l1.inner_border.get(s1);
        double t2_in = l2.inner_border.get(s2);
        odr::Vec3D p1_in = road1->get_xyz(s1, t1_in, 0);
        odr::Vec3D p2_in = road2->get_xyz(s2, t2_in, 0);
#ifdef G_TEST
        EXPECT_LT(odr::euclDistance(p1_in, p2_in), LM::epsilon)
            << road1->id << " " << l1.id << " inner border doesn't meet " << road2->id << " " << l2.id;
#else
        ExpectOrAssert(odr::euclDistance(p1_in, p2_in) < LM::epsilon);
#endif

        auto h1_out = road1->ref_line.get_grad_xy(s1);
        auto h2_out = road2->ref_line.get_grad_xy(s2);
        auto angle_out = std::abs(odr::angle(h1_out, h2_out));
        auto angle_opp_out = std::abs(odr::angle(h1_out, odr::negate(h2_out)));
        ExpectOrAssert(angle_out < LM::epsilon || angle_opp_out < LM::epsilon);
    }

    void Validation::EnsureEndsMeet(const odr::Road* road1, double s1, int lane1,
        const odr::Road* road2, double s2, int lane2)
    {
        odr::Lane l1 = road1->get_lanesection(s1).id_to_lane.at(lane1);
        odr::Lane l2 = road2->get_lanesection(s2).id_to_lane.at(lane2);
        EnsureEndsMeet(road1, s1, l1, road2, s2, l2);
    }

    void Validation::VerifyDirectJunction(const LM::DirectJunction* junction)
    {
        for (auto id2Conn : junction->generated.id_to_connection)
        {
            ExpectOrAssert(id2Conn.second.contact_point != odr::JunctionConnection::ContactPoint_None);
            ExpectOrAssert(id2Conn.second.interface_provider_contact != odr::JunctionConnection::ContactPoint_None);

            auto incoming = IDGenerator::ForType(IDType::Road)->GetByID<LM::Road>(id2Conn.second.incoming_road);
            double sOnIncoming, sOnConnecting;
            ExpectOrAssert(incoming->predecessorJunction.get() == junction
                || incoming->successorJunction.get() == junction);

            if (incoming->predecessorJunction.get() == junction)
            {
                sOnIncoming = 0;
            }
            else if (incoming->successorJunction.get() == junction)
            {
                sOnIncoming = incoming->Length();
            }

            auto connecting = IDGenerator::ForType(IDType::Road)->GetByID<LM::Road>(id2Conn.second.connecting_road);
            ExpectOrAssert(id2Conn.second.contact_point != odr::JunctionConnection::ContactPoint_None);
            if (id2Conn.second.contact_point == odr::JunctionConnection::ContactPoint_Start)
            {
                ExpectOrAssert(connecting->predecessorJunction.get() == junction);
                sOnConnecting = 0;
            }
            else
            {
                ExpectOrAssert(connecting->successorJunction.get() == junction);
                sOnConnecting = connecting->Length();
            }
            
            for (auto laneLink : id2Conn.second.lane_links)
            {
                EnsureEndsMeet(&incoming->generated, sOnIncoming, laneLink.from,
                    &connecting->generated, sOnConnecting, laneLink.to);
            }
        }
    }
}