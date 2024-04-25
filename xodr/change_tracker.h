#pragma once

#include "OpenDriveMap.h"
#include <boost/optional.hpp>

#include <string>
#include <stack>
#include <vector>
namespace RoadRunnerTest { class Validation; }

namespace RoadRunner
{
    class ChangeTracker
    {
        friend class RoadRunnerTest::Validation;
    public:
        ChangeTracker(ChangeTracker&) = delete;
        ChangeTracker& operator=(const ChangeTracker&) = delete;
        static ChangeTracker* Instance();
        
        void StartRecordEdit();
        void FinishRecordEdit();
        bool Undo();
        bool Redo();

        void Save(std::string path);
        bool Load(std::string path);

    private:
        ChangeTracker() {};

        static ChangeTracker* instance;

        odr::OpenDriveMap odrMap;

        struct RoadChange
        {
            boost::optional<odr::Road> before;
            boost::optional<odr::Road> after;
        };

        struct JunctionChange
        {
            boost::optional<odr::Junction> before;
            boost::optional<odr::Junction> after;
        };

        struct MapChange
        {
            std::vector<RoadChange> roadChanges;
            std::vector< JunctionChange> junctionChanges;
        };

        std::stack<MapChange> undoStack;
        std::stack<MapChange> redoStack;

        void RestoreChange(const MapChange& change);
    };
}