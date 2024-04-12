#pragma once

#include <list>
#include <map>
#include <cassert>
#include "spdlog/spdlog.h"

#include "OpenDriveMap.h"
#include "id_generator.h"
#include "road_profile.h"

namespace RoadRunner
{
    class Junction;

    class RoadGraphics;

    class Road: public std::enable_shared_from_this<Road>
    {
    public:
        Road(const RoadProfile& p, std::shared_ptr<odr::RoadGeometry> l);

        Road(const RoadProfile& p, odr::RefLine& l);

        ~Road();

        Road(const Road& another) = delete; // No copy costruct
        Road& operator=(const Road& another) = delete; // No copy assignment

        void Generate(bool notifyJunctions=true);

        /*Without visible change to shape
        * Caller is responsible for re-generate junction
        */
        void ReverseRefLine();

        double Length() const { 
            return generated.ref_line.length;
        }

        void GetEndPoint(bool start, double& x, double& y) const
        {
            double s = start ? 0 : Length();
            auto p = generated.ref_line.get_xy(s);
            x = p[0];
            y = p[1];
        }

        std::string ID() const { return generated.id; }
        odr::RefLine& RefLine() { return generated.ref_line; }
        
        // Static functions
        static int JoinRoads(std::shared_ptr<Road>& road1AsDst, odr::RoadLink::ContactPoint c1,
            std::shared_ptr<Road>& road2ToDel, odr::RoadLink::ContactPoint c2);

        static std::shared_ptr<Road> SplitRoad(std::shared_ptr<Road>& roadAsPrev, double s);

        // Member variables
        odr::Road generated;
        RoadProfile profile;

        std::shared_ptr<Junction> successorJunction, predecessorJunction;

        // Expensive, but safe
        void GenerateAllSectionGraphics();

        // Preferred
        // Sections containing s1/s2 will be updated
        void GenerateOrUpdateSectionGraphicsBetween(double s1, double s2);

        double SnapToSegmentBoundary(double key, double limit);

    private:
        void GenerateSectionGraphicsBetween(double s1, double s2);

        void PlaceOdrRoadMarkings();

        // Prevent formation of too-short segment
        void SnapToSegmentBoundary(type_s& key, type_s limit = 10);

        const double GraphicsDivision = 20;

        const double NeglectableLength = 0.01f;

        // When updates road, remove RoadSectionGraphics then add new
        std::map<double, std::unique_ptr<RoadGraphics>> s_to_section_graphics;
    };
}
