#pragma once

namespace odr { class Road; }
namespace RoadRunner { class Junction; }

namespace RoadRunnerTest
{
    class Validation
    {
    public:
#ifndef G_TEST
        static void ValidateMap();
#endif
        static void VerifySingleRoad(const odr::Road& road);

        static void VerifyJunction(const RoadRunner::Junction* junction);

    private:
#ifndef G_TEST
        static void RoadIDSetMatch();

        static void JunctionIDSetMatch();

        static void VerifyRoadJunctionPtr();
#endif
        static void EnsureEndsMeet(const odr::Road* road1, double s1, int lane1,
            const odr::Road* road2, double s2, int lane2);

        static void VerifyLaneWidthinBound(const odr::Road& road);

        static void VerifySingleRoadLinkage(const odr::Road& road);

        static void VerifyProfileIntegrity(const odr::Road& road);
    };
    
}