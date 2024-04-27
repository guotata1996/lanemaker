#include "change_tracker.h"
#include "id_generator.h"
#include "road.h"
#include "junction.h"
#include "world.h"
#include "test/validation.h"

#include <spdlog/spdlog.h>

namespace RoadRunner
{
    ChangeTracker* ChangeTracker::instance = nullptr;

    ChangeTracker* ChangeTracker::Instance()
    {
        if (instance == nullptr)
        {
            instance = new ChangeTracker;
        }
        return instance;
    }

    void ChangeTracker::PostChangeActions()
    {
        if (VerifyUponChange)
            RoadRunnerTest::Validation::ValidateMap();
    }

    void ChangeTracker::StartRecordEdit()
    {
        IDGenerator::ForRoad()->ClearChangeList();
        IDGenerator::ForJunction()->ClearChangeList();
    }

    void ChangeTracker::FinishRecordEdit()
    {
        auto roadChanges = IDGenerator::ForRoad()->ConsumeChanges();
        auto junctionChanges = IDGenerator::ForJunction()->ConsumeChanges();
        if (roadChanges.empty() && junctionChanges.empty())
        {
            return;
        }

        MapChange recordEntry;

        for (const auto& change : roadChanges)
        {
            std::string id = std::to_string(change.first);
            void* ptr = change.second;

            RoadChange roadChange;
            auto oldIt = odrMap.id_to_road.find(id);
            if (oldIt != odrMap.id_to_road.end())
            {
                spdlog::trace("[Edit] Road {} (len={}) removal recorded", id, oldIt->second.length);
                roadChange.before.emplace(oldIt->second);
                odrMap.id_to_road.erase(id);
            }

            if (ptr != nullptr)
            {
                Road* newRoad = static_cast<Road*>(ptr);
                roadChange.after.emplace(newRoad->generated);
                assert(odrMap.id_to_road.find(newRoad->ID()) == odrMap.id_to_road.end());
                odrMap.id_to_road.emplace(newRoad->ID(), newRoad->generated);
                spdlog::trace("[Edit] Road {} (len={}) creation recorded", id, newRoad->Length());
            }
            recordEntry.roadChanges.push_back(roadChange);
        }
        
        for (const auto& change : junctionChanges)
        {
            std::string id = std::to_string(change.first);
            void* ptr = change.second;

            JunctionChange junctionChange;
            auto oldIt = odrMap.id_to_junction.find(id);
            if (oldIt != odrMap.id_to_junction.end())
            {
                junctionChange.before.emplace(oldIt->second);
                odrMap.id_to_junction.erase(id);
                spdlog::trace("[Edit] Junction {} removal recorded", id);
            }

            if (ptr != nullptr)
            {
                Junction* newJunction = static_cast<Junction*>(ptr);
                junctionChange.after.emplace(newJunction->generated);
                assert(odrMap.id_to_junction.find(newJunction->ID()) == odrMap.id_to_junction.end());
                odrMap.id_to_junction.emplace(newJunction->ID(), newJunction->generated);
                spdlog::trace("[Edit] Junction {} creation recorded", id);
            }
            recordEntry.junctionChanges.push_back(junctionChange);
        }
        
        undoStack.emplace(recordEntry);
        while (!redoStack.empty())
        {
            redoStack.pop();
        }

        PostChangeActions();
    }

    void ChangeTracker::Save(std::string path)
    {
        odrMap.export_file(path);
    }

    bool ChangeTracker::Load(std::string path)
    {
        World::Instance()->allRoads.clear();

        if (!odrMap.Load(path))
        {
            return false;
        }

        // Temporarily hold shared_ptr to Connecting road until they get owned by junction
        std::vector<std::shared_ptr<RoadRunner::Road>> connectingRoadHolder;
        for (const auto& id2Road : odrMap.id_to_road)
        {
            auto rrRoad = std::make_shared<RoadRunner::Road>(id2Road.second);
            rrRoad->GenerateAllSectionGraphics();
            if (id2Road.second.junction == "-1")
            {
                World::Instance()->allRoads.insert(rrRoad);
            }
            else
            {
                connectingRoadHolder.push_back(rrRoad);
            }
        }
        
        // Temporarily hold shared_ptr to Junction until they get owned by connected road
        std::map<std::string, std::shared_ptr<RoadRunner::Junction>> id2RRJunction;

        // Link connecting road <-> junction
        for (const auto& id2Junction : odrMap.id_to_junction)
        {
            auto rrJunc = std::make_shared<RoadRunner::Junction>(id2Junction.second);
            id2RRJunction.emplace(id2Junction.first, rrJunc);
            // Link connecting road to junctions
            for (const auto& id2Connection : id2Junction.second.id_to_connection)
            {
                auto connectingRoadID = id2Connection.second.connecting_road;
                auto roadPtr = static_cast<RoadRunner::Road*>(IDGenerator::ForRoad()->GetByID(connectingRoadID));
                rrJunc->connectingRoads.push_back(roadPtr->shared_from_this());
            }
        }
        // Link connected road <-> pred/succ junction
        for (auto& road : World::Instance()->allRoads)
        {
            if (road->generated.successor.type == odr::RoadLink::Type_Junction)
            {
                auto rrJunction = id2RRJunction.at(road->generated.successor.id);
                rrJunction->AttachNoRegenerate(RoadRunner::ConnectionInfo{ road, odr::RoadLink::ContactPoint_End });
            }
            if (road->generated.predecessor.type == odr::RoadLink::Type_Junction)
            {
                auto rrJunction = id2RRJunction.at(road->generated.predecessor.id);
                rrJunction->AttachNoRegenerate(RoadRunner::ConnectionInfo(road, odr::RoadLink::ContactPoint_Start));
            }
        }

        while (!redoStack.empty())
        {
            redoStack.pop();
        }
        while (!undoStack.empty())
        {
            undoStack.pop();
        }

        PostChangeActions();
        return true;
    }

    bool ChangeTracker::Undo()
    {
        if (undoStack.empty())
        {
            return false;
        }
        auto lastRedo = undoStack.top();
        undoStack.pop();
        MapChange thisTimeChange;

        for (const auto& change : lastRedo.roadChanges)
        {
            thisTimeChange.roadChanges.push_back(RoadChange{ change.after, change.before });
        }
        for (const auto& change : lastRedo.junctionChanges)
        {
            thisTimeChange.junctionChanges.push_back(JunctionChange{ change.after, change.before });
        }

        redoStack.emplace(thisTimeChange);
        RestoreChange(lastRedo);
        return true;
    }

    bool ChangeTracker::Redo()
    {
        if (redoStack.empty())
        {
            return false;
        }
        auto lastUndo = redoStack.top();
        redoStack.pop();
        MapChange thisTimeChange;

        for (const auto& change : lastUndo.roadChanges)
        {
            thisTimeChange.roadChanges.push_back(RoadChange{ change.after, change.before });
        }
        for (const auto& change : lastUndo.junctionChanges)
        {
            thisTimeChange.junctionChanges.push_back(JunctionChange{ change.after, change.before });
        }

        undoStack.emplace(thisTimeChange);
        RestoreChange(lastUndo);
        return true;
    }

    void ChangeTracker::RestoreChange(const MapChange& change)
    {
        // Tear down after
        for (const auto& change : change.roadChanges)
        {
            if (change.after.has_value())
            {
                odrMap.id_to_road.erase(change.after->id);

                auto& odrRoad = change.after.get();
                if (odrRoad.junction == "-1")
                {
                    auto createdID = odrRoad.id;
                    auto roadPtr = static_cast<RoadRunner::Road*>(IDGenerator::ForRoad()->GetByID(createdID))->shared_from_this();
                    World::Instance()->allRoads.erase(roadPtr);

                    if (roadPtr->successorJunction != nullptr)
                    {
                        roadPtr->successorJunction->DetachNoRegenerate(roadPtr);
                    }
                    if (roadPtr->predecessorJunction != nullptr)
                    {
                        roadPtr->predecessorJunction->DetachNoRegenerate(roadPtr);
                    }

                    spdlog::trace("Undo::Tear down road {}", createdID);
                }
            }
        }

        std::set<std::shared_ptr<RoadRunner::Road>> roadAffected;
        for (const auto& change : change.junctionChanges)
        {
            if (change.after.has_value())
            {
                odrMap.id_to_junction.erase(change.after->id);

                auto& odrJunction = change.after.get();
                auto createdID = odrJunction.id;
                auto junctionPtrUntyped = IDGenerator::ForJunction()->GetByID(createdID);

                if (junctionPtrUntyped == nullptr)
                {
                    // Junction could have already been destroyed after deleting all connecting roads
                    continue;
                }
                auto junctionPtr = static_cast<RoadRunner::Junction*>(junctionPtrUntyped)->shared_from_this();
                junctionPtr->connectingRoads.clear();

                for (auto road: junctionPtr->StillConnectedRoads())
                {
                    if (road->successorJunction == junctionPtr)
                    {
                        junctionPtr->DetachNoRegenerate(road);
                        roadAffected.insert(road);
                    }
                    if (road->predecessorJunction == junctionPtr)
                    {
                        junctionPtr->DetachNoRegenerate(road);
                        roadAffected.insert(road);
                    }
                }
            }
        }

        // Restore before
        for (const auto& change : change.roadChanges)
        {
            if (change.before.has_value())
            {
                assert(odrMap.id_to_road.find(change.before->id) == odrMap.id_to_road.end());
                odrMap.id_to_road.emplace(change.before->id, change.before.get());

                spdlog::trace("Undo::Restore Road {}", change.before->id);
                auto odrRoad = change.before.get();
                auto rrRoad = std::make_shared<RoadRunner::Road>(odrRoad);
                rrRoad->GenerateAllSectionGraphics();
                roadAffected.insert(rrRoad);
                if (odrRoad.junction == "-1")
                {
                    World::Instance()->allRoads.insert(rrRoad);
                }
            }
        }

        std::vector<std::shared_ptr<RoadRunner::Junction>> junctionRestored;
        for (const auto& change : change.junctionChanges)
        {
            if (change.before.has_value())
            {
                assert(odrMap.id_to_junction.find(change.before->id) == odrMap.id_to_junction.end());
                odrMap.id_to_junction.emplace(change.before->id, change.before.get());

                auto rrJunc = std::make_shared<RoadRunner::Junction>(change.before.get());
                junctionRestored.push_back(rrJunc);
                spdlog::trace("Undo::Restore Junction {}", rrJunc->ID());

                // Link connecting road to junctions
                for (const auto& id2Connection : rrJunc->generated.id_to_connection)
                {
                    auto connectingRoadID = id2Connection.second.connecting_road;
                    auto roadPtr = static_cast<RoadRunner::Road*>(IDGenerator::ForRoad()->GetByID(connectingRoadID));
                    rrJunc->connectingRoads.push_back(roadPtr->shared_from_this());
                }
            }
        }

        // Link connected road <-> pred/succ junction
        for (auto rrRoad: roadAffected)
        {
            auto odrRoad = rrRoad->generated;
            auto rrRoad = static_cast<RoadRunner::Road*>(IDGenerator::ForRoad()->GetByID(odrRoad.id))->shared_from_this();
                
            if (odrRoad.successor.type == odr::RoadLink::Type_Junction &&
                odrRoad.successor.id != "-1")
            {
                auto juncPtr = static_cast<RoadRunner::Junction*>(IDGenerator::ForJunction()->GetByID(odrRoad.successor.id));
                juncPtr->AttachNoRegenerate(RoadRunner::ConnectionInfo(rrRoad, odr::RoadLink::ContactPoint_End));
            }

            if (odrRoad.predecessor.type == odr::RoadLink::Type_Junction &&
                odrRoad.predecessor.id != "-1")
            {
                auto juncPtr = static_cast<RoadRunner::Junction*>(IDGenerator::ForJunction()->GetByID(odrRoad.predecessor.id));
                juncPtr->AttachNoRegenerate(RoadRunner::ConnectionInfo(rrRoad, odr::RoadLink::ContactPoint_Start));
            }
        }

        PostChangeActions();
    }
}