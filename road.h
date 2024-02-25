#pragma once

#include <list>
#include <cassert>
#include "spdlog/spdlog.h"

#include "OpenDriveMap.h"
#include "Geometries/Line.h"
#include "IDGenerator.h"

namespace RoadRunner
{
    constexpr double LaneWidth = 3.25;

    typedef int8_t type_t;
    typedef uint32_t type_s;

    double to_odr_unit(type_s l);
    double to_odr_unit(type_t l);
    type_s from_odr_unit(double l);

    struct SectionProfile
    {
        type_t offsetx2; // follows XODR s definition
        type_t laneCount; // non-negative value

        bool operator == (const SectionProfile& another) const
        {
            return offsetx2 == another.offsetx2 && laneCount == another.laneCount;
        }

        bool operator != (const SectionProfile& another) const
        {
            return offsetx2 != another.offsetx2 || laneCount != another.laneCount;
        }
    };

    class Road;

    class RoadProfile 
    {
        friend class Road;
    public:
        RoadProfile(uint8_t nLanes_Left, int8_t offsetX2_Left, uint8_t nLanes_Right, int8_t offsetX2_Right);

        RoadProfile& operator=(const RoadProfile& other);

        void OverwriteSection(int side, double start, double end, uint8_t nLanes, int8_t offsetX2);
        
        SectionProfile LeftEntrance() const;
        SectionProfile LeftExit() const;
        SectionProfile RightEntrance() const;
        SectionProfile RightExit() const;

        void Apply(double length, odr::Road&) const;

    protected:
        void ConvertSide(bool rightSide,
            std::string roadID,
            type_s length,
            std::map<double, odr::LaneSection>& laneSectionResult, 
            std::map<double, odr::Poly3>& laneOffsetResult) const;

        void RemoveRedundantProfileKeys(int side);

        std::map<double, odr::Poly3> _MakeTransition(
            type_s start_s, type_s end_s,
            type_t start_t2, type_t end_t2, bool rightSide, type_s length) const;

        std::map<double, odr::Poly3> _MakeStraight(type_s start_s, type_s end_s, type_t const_t, 
            bool rightSide, type_s length) const;

        // Convert difference in L/R lane offset into median center lane
        std::map<double, odr::Poly3> _ComputeMedian(
            const std::map<double, odr::Poly3>& leftOffsets,
            const std::map<double, odr::Poly3> rightOffsets, 
            type_s length) const;

        void _MergeSides(odr::Road& rtn,
            const std::map<double, odr::LaneSection>& leftSections,
            const std::map<double, odr::Poly3>& centerWidths,
            const std::map<double, odr::LaneSection>& rightSections,
            type_s length) const;

        const type_s MaxTransitionS = 20 * 100;
        
        std::map<type_s, SectionProfile> leftProfiles, rightProfiles;

        struct TransitionInfo
        {
            type_s cumulativeS;  // front start (right:s=0, left: s=L)
            type_t oldCenter2, newCenter2; // right: positive
            int startLanes, newLanesOnLeft, newLanesOnRight;
            type_s transitionHalfLength;
        };

        void OverwriteSection(int side, type_s start, type_s end, uint8_t nLanes, int8_t offsetX2);
    };

    // Find Road Pointer by IDGenerator::ForRoad()::GetByID(). Pointer is EXPECTED to change!!
    class Road
    {
    public:
        Road(const RoadProfile& p, std::shared_ptr<odr::RoadGeometry> l, std::string id="") :
            generated(id.empty() ? IDGenerator::ForRoad()->GenerateID(this) : id, 0, "-1"),
            profile(p),
            refLine(l)
        {
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
            refLine.swap(other.refLine);
            IDGenerator::ForRoad()->FreeID(other.ID());
            other.generated.id = "";

            generated.id = IDGenerator::ForRoad()->GenerateID(this);
            generated.name = "Road " + generated.id;
        }

        void Generate()
        {
            profile.Apply(Length(), generated);
            generated.ref_line.s0_to_geometry[0] = refLine->clone();
            generated.DeriveLaneBorders();
        }

        /*Without visible change to shape
        * Caller is responsible for re-generate junction
        */
        void ReverseRefLine();

        double Length() const { 
            return refLine->length;
        }

        std::string ID() const { return generated.id; }
        odr::Road generated;

        RoadProfile profile;
        std::shared_ptr<odr::RoadGeometry> refLine;
    };

    enum RoadJoinError;

    // generate refLine: ConnectLines
    // middle section half/half
    // manage start1 / end2 links, if road1 != road2
    /*Returns: error code*/
    int JoinRoads(Road* const road1AsDst, odr::RoadLink::ContactPoint c1,
                   Road* const road2ToDel, odr::RoadLink::ContactPoint c2);

    Road* SplitRoad(Road* const roadAsPrev, double s);
}
