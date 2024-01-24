#pragma once

#include <list>
#include "OpenDriveMap.h"

namespace RoadRunner
{
    typedef int8_t type_t;
    typedef uint32_t type_s;

    struct RoadProfile
    {
        type_t offsetx2; // follows XODR s definition
        type_t laneCount; // non-negative value

        bool operator == (const RoadProfile& another) const
        {
            return offsetx2 == another.offsetx2 && laneCount == another.laneCount;
        }

        bool operator != (const RoadProfile& another) const
        {
            return offsetx2 != another.offsetx2 || laneCount != another.laneCount;
        }
    };

    struct LaneSection
    {
        RoadProfile profile; // follows XODR t definition
        type_s s; // cm

        // odr::Line3D boundary; // for highlight
    };

    class Road
    {
    public:
        Road(std::string id) { roadID = id; }

        void SetLength(type_s a) { length = a; }
        void AddLeftSection(const LaneSection& section);
        void AddRightSection(const LaneSection& section);
        void InsertSection(type_s sBegin, type_s sEnd, const LaneSection& section);
        void RemoveSectionAt(type_s s);
        LaneSection GetSectionAt(type_s s);
        type_s Length() const { return length; }

        // Draw
        //std::list<odr::Vec3D> GetBoundary();
        // Export
        explicit operator odr::Road() const;

    protected:
        void _UpdateBoundary();

        // Includes center Lane (ID=0)
        void ConvertSide(bool rightSide, std::map<double, odr::LaneSection>& laneSectionResult, std::map<double, odr::Poly3>& laneOffsetResult) const;

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

        static double to_odr_unit(type_s l) { return (double)l / 100; }
        static double to_odr_unit(type_t l) { return (double)l / 2 * LaneWidth; }

        std::string roadID;
        type_s length;
        const type_s MaxTransitionS = 20 * 100;
        static double LaneWidth;
        std::list<LaneSection> leftProfiles, rightProfiles;
        //std::list<odr::Vec3D> boundary;

        struct TransitionInfo
        {
            type_s cumulativeS;  // front start (right:s=0, left: s=L)
            type_t oldCenter2, newCenter2; // right: positive
            int startLanes, newLanesOnLeft, newLanesOnRight;
            type_s transitionHalfLength;
        };
    };
}
