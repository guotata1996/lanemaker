#pragma once

#include <string>

namespace odr {
    class Road; class CubicSpline; class Lane;
}
namespace LM {
    class Road;
    class AbstractJunction;
    class Junction;
    class DirectJunction;
}

namespace LTest
{
    class Validation
    {
    public:
#ifndef G_TEST
        static void ValidateMap();

        static void VerifySingleRoadGraphics(const LM::Road& road);
#endif
        static void VerifySingleRoad(const odr::Road& road);

        static void VerifyJunction(LM::AbstractJunction* junction);

        static bool CompareFiles(const std::string& p1, const std::string& p2);

        static void VerifySingleRoadElevation(const odr::CubicSpline&);

    private:
#ifndef G_TEST
        static void RoadIDSetMatch();

        static void JunctionIDSetMatch();

        static void VerifyRoadJunctionPtr();
#endif
        static void EnsureEndsMeet(const odr::Road* road1, double s1, odr::Lane lane1,
            const odr::Road* road2, double s2, odr::Lane lane2);

        static void EnsureEndsMeet(const odr::Road* road1, double s1, int lane1,
            const odr::Road* road2, double s2, int lane2);

        static void VerifyLaneWidthinBound(const odr::Road& road);

        static void VerifySingleRoadLinkage(const odr::Road& road);

        static void VerifyProfileIntegrity(const odr::Road& road);

        static void VerifyCommonJunction(const LM::Junction* junction);

        static void VerifyDirectJunction(const LM::DirectJunction* junction);

        static void VerifyRoadMarking(const odr::Road& road);

        // Only checks edges-on-record; doesn't recognize dead ends
        static void VerifyRoutingGraph();
    };
    
}