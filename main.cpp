#include <fstream>

#include <QApplication>
#include <QPushButton>
#include <QGraphicsView>
#include <string>

#include "road.h"

#include "GameGraphicsScene.h"

#include "verification.h"
#include "test_randomization.h"


int main(int argc, char** argv)
{
    
    spdlog::set_level(spdlog::level::debug);

    RoadRunner::Road road = GenerateConfig(1);
    GenerateAndVerify(road);

    //QApplication app(argc, argv);
    //MyGraphicsScene scene;
    //scene.DrawXodr(test_map);
    //QGraphicsView view(&scene);
    //view.scale(15, 15);
    //view.show();

    // return app.exec();
}