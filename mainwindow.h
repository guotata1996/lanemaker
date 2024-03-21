#pragma once
#include <qwidget.h>

QT_BEGIN_NAMESPACE
class QGraphicsScene;
QT_END_NAMESPACE

//class PropertyPanel : public QWidget
//{
//public slots:
//    void setProfile
//};

class MainWindow : public QWidget
{
public:
    MainWindow(QWidget* parent = nullptr);
private:
    std::unique_ptr<QGraphicsScene> scene;
};

