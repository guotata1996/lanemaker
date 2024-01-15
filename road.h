#include <list>
#include "OpenDriveMap.h"

namespace RoadRunnder
{
    typedef int8_t type_t;
    typedef uint32_t type_s;

    struct RoadProfile
    {
        type_t nl, ol2, or2, nr; // left(right) lanes, left(right) mid offset from ref_line, in half lanes

        bool operator == (const RoadProfile& another) const
        {
            return nl == another.nl && ol2 == another.ol2 && or2 == another.or2 && nr == another.nr;
        }
    };

    struct LaneSection
    {
        RoadProfile profile;
        type_s length; // cm

        // odr::Line3D boundary; // for highlight
    };

    class Road
    {
    public:
        void AddSection(const LaneSection& section);
        void InsertSection(type_s sBegin, type_s sEnd, const LaneSection& section);
        void RemoveSectionAt(type_s s);
        LaneSection GetSectionAt(type_s s);
        type_s Length() const; // cm

        // Draw
        //std::list<odr::Vec3D> GetBoundary();
        // Export
        explicit operator odr::Road() const;

    private:
        void _UpdateBoundary();

        static std::map<double, odr::Poly3> _MakeTransition(
            type_s start_s, type_s end_s,
            type_t start_t2, type_t end_t2);
        static double to_odr_unit(type_s l) { return (double)l / 100; }
        static double to_odr_unit(type_t l) { return (double)l / 2 * LaneWidth; }

        // transition length = min(section.length / 2, maxTransitionS)
        const type_s MaxTransitionS = 20 * 100;
        static double LaneWidth;
        std::list<LaneSection> profiles;
        //std::list<odr::Vec3D> boundary;

        struct TransitionInfo
        {
            type_s cumulativeS;
            type_t oldCenter2, newCenter2; // right: positive
            int startLanes, newLanesOnLeft, newLanesOnRight;
            type_s transitionHalfLength;
        };
    };
}
