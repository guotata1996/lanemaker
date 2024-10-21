#include "vehicle.h"
#include <vector>
#include <map>
#include <QTimer>

#include "id_generator.h"

class VehicleManager : QObject
{
    Q_OBJECT
public:
    VehicleManager(QObject* parent);

    void Begin();

    void End();

private slots:
    void step();

private:
    void Spawn();

    std::map<std::string, std::shared_ptr<Vehicle>> allVehicles;

    std::unordered_map <odr::LaneKey, std::map<double, std::shared_ptr<Vehicle>>> vehiclesOnLane;

    odr::RoutingGraph routingGraph;

    QTimer* timer;

    IDGenerator* idGen;

    const double FPS = 30;
};