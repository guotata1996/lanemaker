#include "OpenDriveMap.h"

#include <qgraphicsitem.h>
class Vehicle
{
public:
    /*Initiate graphics*/
    Vehicle(odr::LaneKey initialLane, double initialLocalS,
        odr::LaneKey destLane, double destS);

    /*Clean graphics*/
    void Clear();

    /*Return false if fail*/
    bool Step(double dt, const odr::OpenDriveMap& map, const odr::RoutingGraph& graph);

private:
    void updateNavigation(const odr::OpenDriveMap& map, const odr::RoutingGraph& routingGraph);

    double s; // inside current lane section
    double currLaneLength;
    double odoMeter;
    
    double tOffset; // non-zero when lane change starts; gradually decreases to zero
    double laneChangeDueS;

    std::vector<odr::LaneKey> navigation;

    const double DefaultVelocity = 15;
    const double MaxSwitchLaneDistance = 50;

    const odr::LaneKey DestLane;
    const double DestS;

    QGraphicsRectItem* graphics;
};