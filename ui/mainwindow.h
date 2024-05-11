#pragma once
#include <qwidget.h>

QT_BEGIN_NAMESPACE
class QGraphicsScene;
class QStatusBar;
QT_END_NAMESPACE

class MainWidget;

class MainWindow : public QWidget
{
public:
    MainWindow(QWidget* parent = nullptr);
private:
    std::unique_ptr<QGraphicsScene> scene;

    QStatusBar* hintStatus;
    QStatusBar* fpsStatus;

    MainWidget* mainWidget;

public slots:
    void setHint(QString);

    void setFPS(QString);

private slots:
    void saveToFile();

    void loadFromFile();

    void undo();

    void redo();

    void verifyMap();

    void toggleAlwaysVerifyMap(bool);
};

