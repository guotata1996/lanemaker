#include "mainwindow.h"
#include <qgraphicsscene.h>
#include <qgraphicsitem.h>
#include "view.h"

#include <QHBoxLayout>

MainWindow::MainWindow(QWidget* parent): QWidget(parent), scene(new QGraphicsScene(this))
{
    setWindowTitle(tr("Road Runner"));

    auto item = new QGraphicsPathItem;
    QPainterPath path;
    path.addEllipse(100, 200, 80, 120);
    item->setPath(path);

    scene->addItem(item);
    
    auto view = new View("Main View");
    view->view()->setScene(scene);

    QHBoxLayout* layout = new QHBoxLayout;
    layout->addWidget(view);
    setLayout(layout);
}