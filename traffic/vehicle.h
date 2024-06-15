#include "OpenDriveMap.h"

#include <qgraphicsitem.h>
class Vehicle
{
public:
    /*Initiate graphics*/
    Vehicle(odr::LaneKey initialLane, double initialLocalS);

    /*Clean graphics*/
    void Clear();

    /*Return false if fail*/
    bool Step(double dt, const odr::OpenDriveMap& map, const odr::RoutingGraph& graph);

private:
    void updateCurrKeyLength(const odr::OpenDriveMap& map);

    odr::LaneKey currKey;
    double currLaneLength; // 0-- invalid
    double s; // inside current lane section
    double odoMeter;
    
    bool checkedForLaneChange;
    double tOffset; // non-zero when lane change starts; gradually decreases to zero

    const double DefaultVelocity = 10;
    const double LaneChangeDistance = 30;

    QGraphicsRectItem* graphics;
};