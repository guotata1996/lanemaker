#include "junction.h"

namespace RoadRunner
{
    bool TryCreateJunction(std::shared_ptr<RoadRunner::Road>, double, double,
        std::weak_ptr<RoadRunner::Road>, double, std::weak_ptr<RoadRunner::Road>, double);

    bool TryCreateBridgeAndTunnel(std::shared_ptr<RoadRunner::Road>, double, double);
}