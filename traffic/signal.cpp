#include "signal.h"
#include "road.h"
#include "vehicle_manager.h"

#include "id_generator.h"

namespace RoadRunner
{
    extern std::string g_PointerRoadID;

    Signal::Signal(const odr::Junction& junction): highlightedPhase(-1)
    {
        for (const auto& id_conn : junction.id_to_connection)
        {
            controllingRoads.insert(id_conn.second.connecting_road);
            for (auto ll : id_conn.second.lane_links)
            {
                odr::LaneKey laneKey(id_conn.second.connecting_road, 0, ll.to);
                for (int phase : id_conn.second.signalPhases)
                {
                    phaseToLanes[phase].push_back(laneKey);
                }
            }
        }

        currPhase = phaseToLanes.size() - 1;
    }

    void Signal::Update(const unsigned long step, std::unordered_map<odr::LaneKey, bool>& allStates)
    {
        if (step % (VehicleManager::FPS * SecondsPerPhase) == 0)
        {
            if (step == 0)
            {
                // Fill in lookup table
                for (const auto& key : phaseToLanes)
                {
                    for (const auto lane: key.second)
                        allStates[lane] = false;
                }
            }
            HighlightRoadsInCurrentPhase(false);
            for (const auto& key : phaseToLanes.at(currPhase))
            {
                allStates[key] = false;
            }
            currPhase = (currPhase + 1) % phaseToLanes.size();

            for (const auto& key : phaseToLanes.at(currPhase))
            {
                allStates[key] = true;
            }
        }

        if (controllingRoads.find(g_PointerRoadID) != controllingRoads.end())
        {
            HighlightRoadsInCurrentPhase(true);
        }
        else
        {
            HighlightRoadsInCurrentPhase(false);
        }
    }

    void Signal::Terminate()
    {
        HighlightRoadsInCurrentPhase(false);
    }

    void Signal::HighlightRoadsInCurrentPhase(bool enable)
    {
        if (enable && highlightedPhase == currPhase ||
            !enable && highlightedPhase == -1)
        {
            return;
        }

        if (enable)
        {
            highlightedPhase = currPhase;
        }
        else
        {
            highlightedPhase = -1;
        }

        for (auto lanesInPhase : phaseToLanes[currPhase])
        {
            static_cast<Road*>(IDGenerator::ForRoad()->GetByID(lanesInPhase.road_id))->ShowGreenLight(enable);
        }
    }
}