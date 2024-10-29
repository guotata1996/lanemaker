#include "vehicle.h"
#include <vector>
#include <map>
#include <QTimer>

#include "id_generator.h"

class VehicleManager : public QObject
{
    Q_OBJECT
public:
    VehicleManager(QObject* parent);

    void Begin();

    void End();

    void TogglePause();

private slots:
    void step();

private:
    void Spawn();

    std::map<std::string, std::shared_ptr<Vehicle>> allVehicles;

    std::unordered_map <odr::LaneKey, std::map<double, std::shared_ptr<Vehicle>>> vehiclesOnLane;

    odr::RoutingGraph routingGraph;

    std::map<odr::LaneKey, std::vector<std::pair<odr::LaneKey, double>>> overlapZones;

    QTimer* timer;

    IDGenerator* idGen;

    const double FPS = 30;
};