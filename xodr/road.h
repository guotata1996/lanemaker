#pragma once

#include <list>
#include <map>
#include <cassert>
#include <optional>
#include "spdlog/spdlog.h"

#include "OpenDriveMap.h"
#include "id_generator.h"
#include "road_profile.h"

namespace RoadRunnerTest { class Validation; }

namespace RoadRunner
{
    class AbstractJunction;

    class SectionGraphics;

    class Road: public std::enable_shared_from_this<Road>
    {
        friend class RoadRunnerTest::Validation;
    public:
        Road(const LaneProfile& p, std::unique_ptr<odr::RoadGeometry> l);

        Road(const LaneProfile& p, odr::RefLine& l);

        Road(const odr::Road& serialized);

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
            auto p = generated.get_xyz(s, 0, 0);
            x = p[0];
            y = p[1];
        }

        std::string ID() const { return generated.id; }
        odr::RefLine& RefLine() { return generated.ref_line; }
        
        // Static functions
        static int JoinRoads(std::shared_ptr<Road>& road1AsDst, const odr::RoadLink::ContactPoint c1,
            std::shared_ptr<Road>& road2ToDel, const odr::RoadLink::ContactPoint c2);

        static std::shared_ptr<Road> SplitRoad(std::shared_ptr<Road>& roadAsPrev, double s);

        bool ModifyProfile(double s1, double s2,
            const LanePlan& newLeftProfile, const LanePlan& newRightProfile);

        double SnapToSegmentBoundary(double key, double limit, bool* outSuccess = nullptr);

        // Special markings for normal junction
        void UpdateArrowGraphics(odr::RoadLink::ContactPoint c, std::map<int, uint8_t> laneToArrow, bool stopLine);


#ifndef G_TEST
        // Expensive, but safe
        void GenerateAllSectionGraphics();

        // Preferred
        // Sections containing s1/s2 will be updated
        void GenerateOrUpdateSectionGraphicsBetween(double s1, double s2);

        // Special markings for direct junction
        void HideBorderMarkingForDJ(odr::RoadLink::ContactPoint, int side, double untilS);
        void EnableBorderMarking(odr::RoadLink::ContactPoint, int side);

        struct RoadsOverlap
        {
            RoadsOverlap(double aSBegin1, double aSEnd1,
                std::weak_ptr<Road> aRoad2, double aSBegin2, double aSEnd2) :
                sBegin1(aSBegin1), sEnd1(aSEnd1),
                road2(aRoad2), sBegin2(aSBegin2), sEnd2(aSEnd2) {}
            double sBegin1, sEnd1;
            std::weak_ptr<Road> road2;
            double sBegin2, sEnd2;
        };

        // Excluded: Overlap w/ Junction connectingRead and DirectJunction linkedRoad
        std::optional<RoadsOverlap> FirstOverlap(double sBegin, double sEnd) const;

        // Everything included
        std::vector<RoadsOverlap> AllOverlaps(double sBegin, double sEnd) const;

        // sWithin2 is expected to stay within overlapping section against target
        RoadsOverlap CalcOverlapWith(std::shared_ptr<Road> target, double sWithin2, double sBegin1, double sEnd1) const;

        void EnableHighlight(bool enabled);
#endif

        // Member variables
        odr::Road generated;

        std::shared_ptr<AbstractJunction> successorJunction, predecessorJunction;

    private:
        // Prevent formation of too-short segment
        bool SnapToSegmentBoundary(type_s& key, type_s limit = 10);

#ifndef G_TEST
        void GenerateSectionGraphicsBetween(double s1, double s2);

        // Determines resolution for collision detection
        const double GraphicsDivision = 5;

        const double NeglectableLength = 0.01f;

        // When updates road, remove RoadSectionGraphics then add new
        std::map<double, std::unique_ptr<SectionGraphics>> s_to_section_graphics;

        bool highlighted = false;
#endif
    };

    enum RoadJoinError
    {
        RoadJoin_Success,
        RoadJoin_SelfLoop,
        RoadJoin_DirNoOutlet
    };
}
