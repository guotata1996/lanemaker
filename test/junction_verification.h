#pragma once

#include "junction.h"
#include <map>
#include "spdlog/spdlog.h"
#include "test_const.h"

#ifdef G_TEST
#include <gtest/gtest.h>
#endif

namespace RoadRunnerTest
{
    void EnsureEndsMeet(const odr::Road& road1, double s1, int lane1,
        const odr::Road& road2, double s2, int lane2)
    {
        odr::Lane l1 = road1.get_lanesection(s1).id_to_lane.at(lane1);
        odr::Lane l2 = road2.get_lanesection(s2).id_to_lane.at(lane2);
        double t1_out = l1.outer_border.get(s1);
        double t2_out = l2.outer_border.get(s2);
        odr::Vec3D p1_out = road1.get_xyz(s1, t1_out, 0);  // Note: This returns global pos. reflane.get_xy returns local pos.
        odr::Vec3D p2_out = road2.get_xyz(s2, t2_out, 0);
#ifdef G_TEST
        EXPECT_LT(odr::euclDistance(p1_out, p2_out), epsilon)
            << road1.id << " " << lane1 << " outer border doesn't meet " << road2.id << " " << lane2
            << "(" << p1_out[0] << " , " << p1_out[1] << ") vs (" << p2_out[0] << " , " << p2_out[1] << ")";
#endif

        double t1_in = l1.inner_border.get(s1);
        double t2_in = l2.inner_border.get(s2);
        odr::Vec3D p1_in = road1.get_xyz(s1, t1_in, 0);
        odr::Vec3D p2_in = road2.get_xyz(s2, t2_in, 0);
#ifdef G_TEST
        EXPECT_LT(odr::euclDistance(p1_in, p2_in), epsilon)
            << road1.id << " " << lane1 << " inner border doesn't meet " << road2.id << " " << lane2;
#endif
    }

    void VerifyJunction(const RoadRunner::Junction& junction)
    {
#ifdef G_TEST
        EXPECT_EQ(junction.generationError, 0);
#endif
        auto allConnections = odr::get_map_values(junction.generated.id_to_connection);
        // Make sure Road<->Junction pointers match what's declared on xodr
        std::set<std::string> connectingIDFromOdr, connectingIDFromRR;
        for (auto odrLink : junction.generated.id_to_connection)
        {
            connectingIDFromOdr.insert(odrLink.second.connecting_road);
            auto incomingRRRoad = static_cast<RoadRunner::Road*>(IDGenerator::ForRoad()->GetByID(odrLink.second.incoming_road));
            bool incomingContactEnd = incomingRRRoad->successorJunction == junction.shared_from_this();
            bool incomingContactStart = incomingRRRoad->predecessorJunction == junction.shared_from_this();
#ifdef G_TEST
            EXPECT_TRUE(incomingContactEnd || incomingContactStart);
#endif
            if (incomingContactEnd)
            {
                RoadRunner::ConnectionInfo info(incomingRRRoad->shared_from_this(), odr::RoadLink::ContactPoint_End);
                EXPECT_TRUE(junction.formedFrom.find(info) != junction.formedFrom.end());
            }
            if (incomingContactStart)
            {
                RoadRunner::ConnectionInfo info(incomingRRRoad->shared_from_this(), odr::RoadLink::ContactPoint_Start);
                EXPECT_TRUE(junction.formedFrom.find(info) != junction.formedFrom.end());
            }

            auto outgoingRRRoad = static_cast<RoadRunner::Road*>(IDGenerator::ForRoad()->GetByID(odrLink.second.outgoing_road));
            bool outgoingContactEnd = outgoingRRRoad->successorJunction == junction.shared_from_this();
            bool outgoingContactStart = outgoingRRRoad->predecessorJunction == junction.shared_from_this();
#ifdef G_TEST
            EXPECT_TRUE(outgoingContactEnd || outgoingContactStart);
#endif
            if (outgoingContactEnd)
            {
                RoadRunner::ConnectionInfo info(outgoingRRRoad->shared_from_this(), odr::RoadLink::ContactPoint_End);
                EXPECT_TRUE(junction.formedFrom.find(info) != junction.formedFrom.end());
            }
            if (outgoingContactStart)
            {
                RoadRunner::ConnectionInfo info(outgoingRRRoad->shared_from_this(), odr::RoadLink::ContactPoint_Start);
                EXPECT_TRUE(junction.formedFrom.find(info) != junction.formedFrom.end());
            }
        }
        for (auto road : junction.connectingRoads)
        {
            connectingIDFromRR.insert(road->ID());
        }
#ifdef G_TEST
        EXPECT_EQ(connectingIDFromOdr, connectingIDFromRR);
#else
        assert(connectingIDFromOdr == connectingIDFromRR);
#endif

        // Make sure all incoming roads' entering lanes have matching connectings
        for (auto& incomingInfo : junction.formedFrom)
        {
            const odr::Road& incomingRoad = incomingInfo.road.lock()->generated;
            auto link = incomingInfo.contact == odr::RoadLink::ContactPoint_Start ? incomingRoad.predecessor : incomingRoad.successor;
#ifdef G_TEST
            EXPECT_EQ(link.type, odr::RoadLink::Type_Junction);
            EXPECT_EQ(link.id, junction.generated.id);
#endif
            std::map<int, std::pair<std::string, int>> incomingLaneToConnectingRoadLane;
            for (const auto& connection : allConnections)
            {
                if (connection.incoming_road == incomingRoad.id)
                {
#ifdef G_TEST
                    EXPECT_EQ(connection.contact_point, odr::JunctionConnection::ContactPoint_Start);
#endif
                    for (const auto& ll : connection.lane_links)
                    {
                        incomingLaneToConnectingRoadLane.emplace(ll.from, std::make_pair(connection.connecting_road, ll.to));
                    }
                }
            }

            auto enteringLanes = incomingRoad.s_to_lanesection.begin()->second.get_sorted_driving_lanes(
                incomingInfo.contact == odr::RoadLink::ContactPoint_Start ? 1 : -1);
            for (const odr::Lane& enteringLane : enteringLanes)
            {
                auto connectingRoadAndLane = incomingLaneToConnectingRoadLane.at(enteringLane.id);
                std::string connectingRoadID = connectingRoadAndLane.first;
                int connectingLane = connectingRoadAndLane.second;
                auto connectingRoad = std::find_if(junction.connectingRoads.begin(), junction.connectingRoads.end(),
                    [connectingRoadID](auto& road) {return road->ID() == connectingRoadID; });

                double incomingS = incomingInfo.contact == odr::RoadLink::ContactPoint_Start ? 0 : incomingRoad.length;
                EnsureEndsMeet(incomingRoad, incomingS, enteringLane.id,
                    (*connectingRoad)->generated, 0, connectingLane);
            }
        }

        // Make sure all connecting roads have matching outgoing lanes
        for (auto& connecting : junction.connectingRoads)
        {
            auto outLink = connecting->generated.successor;
#ifdef G_TEST
            EXPECT_EQ(connecting->generated.junction, junction.generated.id);
            EXPECT_EQ(outLink.type, odr::RoadLink::Type_Road);
#endif
            std::string outgoingID = outLink.id;
            std::string connectingID = connecting->ID();
            auto connection = std::find_if(allConnections.begin(), allConnections.end(),
                [connectingID](auto& connection) {return connectingID == connection.connecting_road; });
#ifdef G_TEST
            EXPECT_EQ(connection->outgoing_road, outgoingID);
#endif
            auto contactPointOnNext = connection->lane_links.begin()->next < 0 ?
                odr::RoadLink::ContactPoint_Start : odr::RoadLink::ContactPoint_End;
            auto outgoingItr = std::find_if(junction.formedFrom.begin(), junction.formedFrom.end(),
                [outgoingID, contactPointOnNext](const RoadRunner::ConnectionInfo& info) 
            {
                return info.road.lock()->ID() == outgoingID && info.contact == contactPointOnNext;
            });
            for (const auto& ll : connection->lane_links)
            {
                double outgoingS = outgoingItr->contact == odr::RoadLink::ContactPoint_Start ? 0 : outgoingItr->road.lock()->Length();
                EnsureEndsMeet(connecting->generated, connecting->generated.length, ll.to,
                    outgoingItr->road.lock()->generated, outgoingS, ll.next);
            }
        }
    }
}