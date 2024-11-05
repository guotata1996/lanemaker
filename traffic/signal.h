#pragma once

#include "Junction.h"

#include <map>
#include <set>
#include <vector>

namespace RoadRunner
{
    class Signal
    {
    public:
        Signal(const odr::Junction&);

        void Update(const unsigned long step, std::unordered_map<odr::LaneKey, bool>& allStates);

        void Terminate();

    private:
        void HighlightRoadsInCurrentPhase(bool enabled);

        std::map<int, std::vector< odr::LaneKey>> phaseToLanes;

        std::set<std::string> controllingRoads;

        int currPhase;

        int highlightedPhase;

        const int SecondsPerPhase = 10;
    };
};
