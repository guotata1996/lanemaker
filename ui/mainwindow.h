#pragma once
#include <qwidget.h>

QT_BEGIN_NAMESPACE
class QGraphicsScene;
class QStatusBar;
class ReplayWindow;
QT_END_NAMESPACE

class MainWidget;
class VehicleManager;

class MainWindow : public QWidget
{
public:
    MainWindow(QWidget* parent = nullptr);

    ~MainWindow();
private:
    std::unique_ptr<QGraphicsScene> scene;

    std::unique_ptr<QStatusBar> hintStatus;
    std::unique_ptr<QStatusBar> fpsStatus;

    std::unique_ptr<MainWidget> mainWidget;
    std::unique_ptr<VehicleManager> vehicleManager;

    std::unique_ptr<ReplayWindow> replayWindow;

    QAction* toggleSimAction;

    bool quitReplayComplete;

public slots:
    void ReplaySingleStep();

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

    void playActionHistory();

    void toggleSimulation(bool);

    void enableSimulation(bool);

    void onAppQuit();

    void onReplayDone(bool);

private:
    void openReplayWindow(bool playImmediate);
};

