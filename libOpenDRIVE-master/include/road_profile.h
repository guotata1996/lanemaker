#pragma once
#include <map>
#include <set>

#include "Geometries/CubicSpline.h"
#include "LaneSection.h"

namespace odr
{
    class Road;
    class OpenDriveMap;
}

namespace RoadRunnerTest
{
class Validation;
}

namespace RoadRunner
{
    typedef int8_t   type_t;
    typedef uint32_t type_s;

    constexpr double LaneWidth = 3.25;
    constexpr double MaxTransition = 20;
    const type_s     MaxTransitionS = MaxTransition * 100;

    double to_odr_unit(type_s l);
    double to_odr_unit(type_t l);
    type_s from_odr_unit(double l);

    constexpr type_s ProfileMinLengthCM = 100;

    struct LanePlan
    {
        type_t offsetx2; // follows XODR s definition
        type_t laneCount; // non-negative value

        bool operator == (const LanePlan& another) const
        {
            return offsetx2 == another.offsetx2 && laneCount == another.laneCount;
        }

        bool operator != (const LanePlan& another) const
        {
            return offsetx2 != another.offsetx2 || laneCount != another.laneCount;
        }
    };

    //typedef int8_t ElevationPlan;

    class LaneProfile
    {
        friend class odr::OpenDriveMap;
        friend class RoadRunnerTest::Validation;
    public:
        LaneProfile() = default;

        LaneProfile(uint8_t nLanes_Left, int8_t offsetX2_Left, uint8_t nLanes_Right, int8_t offsetX2_Right);

        LaneProfile& operator=(const LaneProfile& other);

        void OverwriteSection(double start, double end, double length,
            const LanePlan& newLeftProfile, const LanePlan& newRightProfile);

        /*First section travelling on left side, @s = Length*/
        LanePlan LeftEntrance() const;
        /*Last section travelling on left side, @s = 0*/
        LanePlan LeftExit() const;
        /*First section travelling on right side, @s = 0*/
        LanePlan RightEntrance() const;
        /*Last section travelling on right side, @s = Length*/
        LanePlan RightExit() const;

        bool HasSide(int side) const;

        void Apply(double length, odr::Road*);

        LanePlan ProfileAt(double s, int side) const;
        
        bool SnapToSegmentBoundary(type_s& key, type_s length, type_s limit = 10) const;

        LaneProfile Reversed(type_s length) const;

        void Split(type_s length, type_s splitPoint, LaneProfile& p1, LaneProfile& p2) const;

        void Join(type_s originalLength, type_s road2Base, type_s road2Length, 
            const LaneProfile& road2Profile, type_s finalLength);

        std::string ToString() const;

    protected:
        std::map<type_s, LanePlan> leftPlans, rightPlan;

        // starts at 0, ends at length
        std::set<type_s> GetAllKeys(type_s length) const;

        // right side keys are in (s_small, s_big)
        // left side keys are in (s_big, s_small)
        // begins with 0, ends at length
        std::map<std::pair<type_s, type_s>, LanePlan> GetAllSections(type_s length, int side) const;

        void OverwriteSection(int side, type_s start, type_s end, uint8_t nLanes, int8_t offsetX2);

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

        void _MergeSides(odr::Road* rtn,
            const std::map<double, odr::LaneSection>& leftSections,
            const std::map<double, odr::Poly3>& centerWidths,
            const std::map<double, odr::LaneSection>& rightSections,
            type_s length) const;

        struct TransitionInfo
        {
            type_s cumulativeS;  // front start (right:s=0, left: s=L)
            type_t oldCenter2, newCenter2; // right: positive
            int startLanes, newLanesOnLeft, newLanesOnRight;
            type_s transitionHalfLength;
        };
    };

    class CubicSplineGenerator
    {
    public:
        static void OverwriteSection(odr::CubicSpline& target, double length, double start, double end, double value);

        static double MaxTransitionLength;
    private:
        // f(0) = y0, f(len) = y1, f'(0) = f'(len) = 0
        static odr::Poly3 FitPoly3(double s0, double y0, double s1, double y1);
    };
}