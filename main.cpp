#include <fstream>


#include <QApplication>
#include <QPushButton>
#include <QGraphicsView>

#include <string>

#include "GameGraphicsScene.h"


int main(int argc, char** argv)
{
    odr::OpenDriveMap odr_map("C:\\Users\\guota\\Downloads\\Town06.xodr");


    QApplication app(argc, argv);
    //QPushButton button(std::to_string(edges).c_str());
    //button.show();

    MyGraphicsScene scene;
    scene.DrawXodr(odr_map);
    // scene.addSimpleText(QString::fromStdString(std::to_string(faces)));
    QGraphicsView view(&scene);
    view.scale(5, 5);
    view.show();

    return app.exec();
}