#include "Road.h"

namespace RoadRunner
{
    bool borderIntersect(const odr::Road& roadA, const int sideA,
        const odr::Road& roadB, const int sideB, double& outSA, double& outSB,
        const odr::RoadLink::ContactPoint searchBeginA = odr::RoadLink::ContactPoint_None,
        const odr::RoadLink::ContactPoint searchBeginB = odr::RoadLink::ContactPoint_None,
        const double SearchLimit = 200.0);
}