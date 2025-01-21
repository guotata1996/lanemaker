#include "OpenDriveMap.h"

#include <qgraphicsitem.h>
#include <optional>

class Vehicle: public std::enable_shared_from_this<Vehicle>
{
public:
    /*Initiate graphics*/
    Vehicle(odr::LaneKey initialLane, double initialLocalS, odr::LaneKey destLane, double destS, double maxV = 20);

    void InitGraphics();

    bool GotoNextGoal(const odr::OpenDriveMap& odrMap, const odr::RoutingGraph& routingGraph,
        const std::unordered_map<odr::LaneKey, int>& trafficInfo);

    /*Clean graphics*/
    void Clear();

    void EnableRouteVisual(bool enabled, const odr::OpenDriveMap& odrMap);

    /*Return false if fail
    * Only use others' last frame info, DO NOT use any of new_ info
    */
    bool PlanStep(double dt, const odr::OpenDriveMap& map,
        const std::unordered_map<odr::LaneKey, 
        std::map<double, std::shared_ptr<Vehicle>>>& vehiclesOnLane,
        const std::map<odr::LaneKey, std::vector<std::pair<odr::LaneKey, double>>>& overlapZones,
        const std::unordered_map<odr::LaneKey, bool>& signalStates);
    
    void MakeStep(double dt, const odr::OpenDriveMap& map);

    double S() const;
    double V() const;
    std::vector<odr::LaneKey> OccupyingLanes() const; // 2 (parallel lanes) when lane switching
    odr::Vec3D TipPos() const;
    odr::Vec3D TailPos() const;

    static std::shared_ptr<Vehicle> GetLeaderInOverlapZone(
        odr::LaneKey lane, double s,
        const odr::OpenDriveMap& map,
        const std::unordered_map<odr::LaneKey, std::map<double, std::shared_ptr<Vehicle>>>& vehiclesOnLane,
        const std::map<odr::LaneKey, std::vector<std::pair<odr::LaneKey, double>>>& overlapZoneInfo,
        double& outDistance, double lookforward = 50);

    std::shared_ptr<const Vehicle> GetLeader(const odr::OpenDriveMap& map,
        const std::unordered_map<odr::LaneKey, std::map<double, std::shared_ptr<Vehicle>>>& vehiclesOnLane,
        const std::map<odr::LaneKey, std::vector<std::pair<odr::LaneKey, double>>>& overlapZoneInfo,
        const std::unordered_map<odr::LaneKey, bool>& signalStates,
        double& outDistance, double lookforward = 50) const;

    double vFromGibbs(double dt, std::shared_ptr<const Vehicle> leader, double distance) const;

    std::string Log();

    const std::string ID;

private:
    void updateNavigation(const odr::OpenDriveMap& map, const odr::RoutingGraph& routingGraph,
        const std::unordered_map<odr::LaneKey, int>& nVehiclesOnlane);

    double new_velocity, new_s;

    double s; // inside current lane section
    double velocity;

    double currLaneLength;
    double tOffset; // non-zero when lane change starts; gradually decreases to zero
    double laneChangeDueS;

    std::vector<odr::LaneKey> navigation;
    std::optional<odr::LaneKey> lcFrom;  // active during a lane change

    const double MaxSwitchLaneDistance = 50;
    const double LCCompleteThreshold = 0.2;
    const odr::LaneKey ALane, BLane;
    const double AS, BS;

    bool goalIndex;
    unsigned long stepInJunction;
    odr::LaneKey sourceLane() const;
    odr::LaneKey destLane() const;
    double sourceS() const;
    double destS() const;
    const double MaxV;
    const unsigned long DestroyIfInJunction = 30 * 30;

    odr::Vec3D position;
    double heading;
    static QVector3D DimensionLWH;
    
    std::vector<unsigned int> routeVisualIndex;
    odr::Line3D leaderLine;

    const unsigned int variation;
};