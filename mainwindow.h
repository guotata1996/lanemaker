#pragma once
#include <qwidget.h>

QT_BEGIN_NAMESPACE
class QGraphicsScene;
QT_END_NAMESPACE

class MainWindow :
    public QWidget
{
public:
    MainWindow(QWidget* parent = nullptr);

private:
    QGraphicsScene* scene;
};

