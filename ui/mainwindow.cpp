#include "mainwindow.h"
#include <qgraphicsscene.h>
#include <qgraphicsitem.h>
#include "view.h"

#include <QHBoxLayout>

QGraphicsScene* g_scene;

MainWindow::MainWindow(QWidget* parent): QWidget(parent)
{
    setWindowTitle(tr("Road Runner"));
    setMinimumWidth(1600);
    setMinimumHeight(900);

    scene = std::make_unique<QGraphicsScene>(this);
    g_scene = scene.get();
    
    auto view = new View("Main View");
    view->view()->setScene(g_scene);

    auto mainLayout = new QHBoxLayout;
    mainLayout->addWidget(view);
    
    setLayout(mainLayout);
}