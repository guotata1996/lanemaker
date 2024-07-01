#pragma once
#include <qwidget.h>

QT_BEGIN_NAMESPACE
class QGraphicsScene;
class QStatusBar;
QT_END_NAMESPACE

class MainWidget;
class VehicleManager;

class MainWindow : public QWidget
{
public:
    MainWindow(QWidget* parent = nullptr);

    ~MainWindow();
private:
    std::string DefaultSaveFolder() const;

    std::unique_ptr<QGraphicsScene> scene;

    std::unique_ptr<QStatusBar> hintStatus;
    std::unique_ptr<QStatusBar> fpsStatus;

    std::unique_ptr<MainWidget> mainWidget;
    std::unique_ptr<VehicleManager> vehicleManager;

    QAction* toggleSimAction;

private slots:
    void setHint(QString);

    void setFPS(QString);

    void newMap();

    void saveToFile();

    void loadFromFile();

    void undo();

    void redo();

    void verifyMap();

    void toggleAlwaysVerifyMap(bool);

    void saveActionHistory();

    void debugActionHistory();

    void toggleSimulation(bool);

    void enableSimulation(bool);

    void onAppQuit();
};

