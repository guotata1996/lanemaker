#pragma once

#include <list>
#include <cassert>
#include "spdlog/spdlog.h"

#include "OpenDriveMap.h"
#include "IDGenerator.h"
#include "roadProfile.h"

namespace RoadRunner
{
    // Find Road Pointer by IDGenerator::ForRoad()::GetByID(). Pointer is EXPECTED to change!!
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

        ~Road() 
        {
            if (!ID().empty())
            {
                spdlog::trace("del road {}", ID());
                IDGenerator::ForRoad()->FreeID(ID());
            }  
        }

        Road(const Road& another) = delete; // No copy costruct
        Road& operator=(const Road& another) = delete; // No copy assignment

        // Move constructor
        Road(Road&& other) noexcept :
            generated(other.generated),
            profile(other.profile)
        {
            IDGenerator::ForRoad()->FreeID(other.ID());
            other.generated.id = "";

            generated.id = IDGenerator::ForRoad()->GenerateID(this);
            generated.name = "Road " + generated.id;
        }

        void Generate()
        {
            profile.Apply(Length(), generated);
            generated.DeriveLaneBorders();
            IDGenerator::ForRoad()->NotifyChange(ID());
        }

        /*Without visible change to shape
        * Caller is responsible for re-generate junction
        */
        void ReverseRefLine();

        double Length() const { 
            return generated.ref_line.length;
        }

        std::string ID() const { return generated.id; }
        odr::RefLine& RefLine() { return generated.ref_line; }
        
        odr::Road generated;
        RoadProfile profile;

        enum RoadJoinError;

        static int JoinRoads(std::shared_ptr<Road>& road1AsDst, odr::RoadLink::ContactPoint c1,
            std::shared_ptr<Road>& road2ToDel, odr::RoadLink::ContactPoint c2);

        static std::unique_ptr<Road> SplitRoad(std::shared_ptr<Road>& roadAsPrev, double s);
    };

    

    // generate refLine: ConnectLines
    // middle section half/half
    // manage start1 / end2 links, if road1 != road2
    /*Returns: error code*/
    

    
}
