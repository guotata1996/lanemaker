#pragma once

#include "OpenDriveMap.h"
#include <boost/optional.hpp>

#include <string>
namespace RoadRunner
{
    class ChangeTracker
    {
    public:
        ChangeTracker(ChangeTracker&) = delete;
        ChangeTracker& operator=(const ChangeTracker&) = delete;
        static ChangeTracker* Instance();

        void Record();
        bool Undo();
        bool Redo();

        void Save(std::string path);
        void load(std::string path);

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

        std::vector<MapChange> undoStack;
        std::vector<MapChange> redoStack;
    };
}