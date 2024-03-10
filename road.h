#pragma once

#include <list>
#include <cassert>
#include "spdlog/spdlog.h"

#include "OpenDriveMap.h"
#include "IDGenerator.h"
#include "roadProfile.h"

namespace RoadRunner
{
    class Junction;

    class Road
    {
    public:
        Road(const RoadProfile& p, std::shared_ptr<odr::RoadGeometry> l) :
            generated(IDGenerator::ForRoad()->GenerateID(this), 0, "-1"),
            profile(p)
        {
            generated.ref_line.length = l->length;
            generated.ref_line.s0_to_geometry.emplace(0, l->clone());
            Generate();
        }

        Road(const RoadProfile& p, odr::RefLine& l) :
            generated(IDGenerator::ForRoad()->GenerateID(this), 0, "-1"),
            profile(p)
        {
            generated.ref_line = std::move(l);
            Generate();
        }

        ~Road();

        Road(const Road& another) = delete; // No copy costruct
        Road& operator=(const Road& another) = delete; // No copy assignment

        void Generate();

        /*Without visible change to shape
        * Caller is responsible for re-generate junction
        */
        void ReverseRefLine();

        double Length() const { 
            return generated.ref_line.length;
        }

        std::string ID() const { return generated.id; }
        odr::RefLine& RefLine() { return generated.ref_line; }
        
        // Static functions
        static int JoinRoads(std::shared_ptr<Road>& road1AsDst, odr::RoadLink::ContactPoint c1,
            std::shared_ptr<Road>& road2ToDel, odr::RoadLink::ContactPoint c2);

        static std::unique_ptr<Road> SplitRoad(std::shared_ptr<Road>& roadAsPrev, double s);

        // Member variables
        odr::Road generated;
        RoadProfile profile;

        std::shared_ptr<Junction> successorJunction, predecessorJunction;
    };
}
