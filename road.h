#pragma once

#include <list>
#include <cassert>
#include "OpenDriveMap.h"
#include "Geometries/Line.h"

namespace RoadRunner
{
    constexpr double LaneWidth = 3.25;

    typedef int8_t type_t;
    typedef uint32_t type_s;

    double to_odr_unit(type_s l);
    double to_odr_unit(type_t l);

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

    struct LaneSection
    {
        SectionProfile profile; // follows XODR t definition
        type_s s; // cm

        // odr::Line3D boundary; // for highlight
    };

    class RoadProfile 
    {
    public:
        RoadProfile(type_s l = 0) { length = l; }
        void SetLength(type_s a) { length = a; }

        void AddLeftSection(const LaneSection& section);
        void AddRightSection(const LaneSection& section);
        
        SectionProfile LeftEntrance() const;
        SectionProfile LeftExit() const;
        SectionProfile RightEntrance() const;
        SectionProfile RightExit() const;

        type_s Length() const { return length; }

        void Apply(odr::Road&) const;

    protected:
        // Includes center Lane (ID=0)
        void ConvertSide(bool rightSide,
            std::string roadID,
            std::map<double, odr::LaneSection>& laneSectionResult, 
            std::map<double, odr::Poly3>& laneOffsetResult) const;

        std::map<double, odr::Poly3> _MakeTransition(
            type_s start_s, type_s end_s,
            type_t start_t2, type_t end_t2, bool rightSide) const;

        std::map<double, odr::Poly3> _MakeStraight(type_s start_s, type_s end_s, type_t const_t, bool rightSide) const;

        // Convert difference in L/R lane offset into median center lane
        std::map<double, odr::Poly3> _ComputeMedian(const std::map<double, odr::Poly3>& leftOffsets,
            const std::map<double, odr::Poly3> rightOffsets) const;

        void _MergeSides(odr::Road& rtn,
            const std::map<double, odr::LaneSection>& leftSections,
            const std::map<double, odr::Poly3>& centerWidths,
            const std::map<double, odr::LaneSection>& rightSections) const;

        type_s length;
        const type_s MaxTransitionS = 20 * 100;
        
        std::list<LaneSection> leftProfiles, rightProfiles;

        struct TransitionInfo
        {
            type_s cumulativeS;  // front start (right:s=0, left: s=L)
            type_t oldCenter2, newCenter2; // right: positive
            int startLanes, newLanesOnLeft, newLanesOnRight;
            type_s transitionHalfLength;
        };
    };

    class Road
    {
    public:
        Road(std::string id, const RoadProfile& p) :
            generated(id, 0, "", "Road " + id),
            profile(p) {}

        void Generate(const odr::RoadGeometry& refLine)
        {
            profile.SetLength(refLine.length * 100);
            profile.Apply(generated);
            generated.ref_line.s0_to_geometry[0] = refLine.clone();
        }

        void Generate()
        {
            assert(profile.Length() > 0);
            profile.Apply(generated);
            generated.ref_line.s0_to_geometry[0] = std::make_unique<odr::Line>(0, 0, 0, 0, generated.length);
        }

        double Length() const { 
            return to_odr_unit(profile.Length()); 
        }

        std::string ID() const { return generated.id; }

        RoadProfile profile;
        // TODO: ref line fitter
        odr::Road generated;
    };
}
