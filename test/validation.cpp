#include "validation.h"

#include "id_generator.h"
#include "world.h"
#include "road.h"
#include "junction.h"

#ifdef G_TEST
    #include <gtest/gtest.h>
#else
    #include "change_tracker.h"
#endif

namespace RoadRunnerTest
{
#ifndef G_TEST
    void Validation::ValidateMap()
    {
        RoadIDSetMatch();
        JunctionIDSetMatch();
        VerifyRoadJunctionPtr();

        // Each junction geometry okay
        for (auto idAndJunction : IDGenerator::ForJunction()->assignTo)
        {
            auto junc = static_cast<RoadRunner::AbstractJunction*>(idAndJunction.second);
            VerifyJunction(junc);
        }

        for (auto road : World::Instance()->allRoads)
        {
            VerifySingleRoad(road->generated);
        }
        spdlog::info("Passed Verification!");
    }

    // road IDs match among IDGenerator | world | odrMap
    void Validation::RoadIDSetMatch()
    {
        const auto& serializedMap = RoadRunner::ChangeTracker::Instance()->odrMap;
        auto world = World::Instance();

        std::set<std::string> roadIDsFromSerialized;
        std::set<std::string> nonConnRoadIDsFromSerialized;

        for (auto idAndRoad : serializedMap.id_to_road)
        {
            assert(idAndRoad.first == idAndRoad.second.id);
            roadIDsFromSerialized.insert(idAndRoad.first);
            auto roadPtr = static_cast<RoadRunner::Road*>(IDGenerator::ForRoad()->GetByID(idAndRoad.first));
            assert(roadPtr != nullptr);
            if (idAndRoad.second.junction == "-1")
            {
                nonConnRoadIDsFromSerialized.insert(idAndRoad.first);
            }
        }

        std::set<std::string> nonConnRoadIDsFromWorld;
        for (const auto& road : world->allRoads)
        {
            nonConnRoadIDsFromWorld.insert(road->ID());
        }

        std::set<std::string> roadIDsFromIDGenerator;
        for (auto idAndRoad : IDGenerator::ForRoad()->assignTo)
        {
            roadIDsFromIDGenerator.insert(std::to_string(idAndRoad.first));
        }
        assert(roadIDsFromSerialized == roadIDsFromIDGenerator);
        assert(nonConnRoadIDsFromSerialized == nonConnRoadIDsFromWorld);
    }

    // Junction IDs match between IDGenerator | odrMap
    void Validation::JunctionIDSetMatch()
    {
        const auto& serializedMap = RoadRunner::ChangeTracker::Instance()->odrMap;
        std::set<std::string> junctionIDsFromSerialized;
        for (auto idAndJunction : serializedMap.id_to_junction)
        {
            assert(idAndJunction.first == idAndJunction.second.id);
            junctionIDsFromSerialized.insert(idAndJunction.first);
            auto juncPtr = static_cast<RoadRunner::AbstractJunction*>(IDGenerator::ForJunction()->GetByID(idAndJunction.first));
            assert(juncPtr != nullptr);
        }

        std::set<std::string> junctionIDsFromIDGenerator;
        for (auto idAndJunction : IDGenerator::ForJunction()->assignTo)
        {
            junctionIDsFromIDGenerator.insert(std::to_string(idAndJunction.first));
        }
        assert(junctionIDsFromSerialized == junctionIDsFromIDGenerator);
    }

    void Validation::VerifyRoadJunctionPtr()
    {
        const auto& serializedMap = RoadRunner::ChangeTracker::Instance()->odrMap;
        for (const auto& idAndRoad : serializedMap.id_to_road)
        {
            auto roadPtr = static_cast<RoadRunner::Road*>(IDGenerator::ForRoad()->GetByID(idAndRoad.first));
            auto serializedRoad = idAndRoad.second;
            if (serializedRoad.predecessor.type == odr::RoadLink::Type_Junction)
            {
                assert(serializedRoad.predecessor.id == roadPtr->predecessorJunction->ID());
            }
            if (serializedRoad.successor.type == odr::RoadLink::Type_Junction)
            {
                assert(serializedRoad.successor.id == roadPtr->successorJunction->ID());
            }
        }
        for (const auto& idAndJunction : serializedMap.id_to_junction)
        {
            auto junctionPtr = static_cast<RoadRunner::AbstractJunction*>(IDGenerator::ForJunction()->GetByID(idAndJunction.first));
            auto commonPtr = dynamic_cast<RoadRunner::Junction*>(junctionPtr);

            if (commonPtr == nullptr)
            {
                // Direct junction has no connecting road
                continue;
            }
            auto serializedJunction = idAndJunction.second;
            std::set<std::string> connectingsFromSerialized;
            for (const auto& idAndConn : serializedJunction.id_to_connection)
            {
                std::string connRoad = idAndConn.second.connecting_road;
                connectingsFromSerialized.insert(connRoad);
            }

            std::set<std::string> connectingsFromPointer;
            for (auto info : commonPtr->connectingRoads)
            {
                connectingsFromPointer.insert(info->ID());
            }
            assert(connectingsFromSerialized == connectingsFromPointer);
        }
    }
#endif
}