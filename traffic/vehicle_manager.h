#include "vehicle.h"
#include <list>
#include <vector>
#include <QTimer>

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

    std::list<Vehicle> allVehicles;

    odr::RoutingGraph routingGraph;

    QTimer* timer;

    const double FPS = 30;
};