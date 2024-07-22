#pragma once
#include <qwidget.h>

QT_BEGIN_NAMESPACE
class QGraphicsScene;
class QStatusBar;
class ReplayWindow;
class PreferenceWindow;
QT_END_NAMESPACE

class MainWidget;
class VehicleManager;

class MainWindow : public QWidget
{
public:
    MainWindow(QWidget* parent = nullptr);

    ~MainWindow();

    void resizeDontRecord(int w, int h);

protected:
    void resizeEvent(QResizeEvent*) override;

    void closeEvent(QCloseEvent* event) override;

private:
    const int StartWidth = 1600;
    const int StartHeight = 1000;

    std::unique_ptr<QGraphicsScene> scene;

    std::unique_ptr<QStatusBar> hintStatus;
    std::unique_ptr<QStatusBar> fpsStatus;

    std::unique_ptr<MainWidget> mainWidget;
    std::unique_ptr<VehicleManager> vehicleManager;

    std::unique_ptr<ReplayWindow> replayWindow;
    std::unique_ptr<PreferenceWindow> preferenceWindow;

    QAction* toggleSimAction;

    bool quitReplayComplete;

    bool recordResize = true;

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

    void saveActionHistory();

    void debugActionHistory();

    void playActionHistory();

    void toggleSimulation(bool);

    void enableSimulation(bool);

    void onReplayDone(bool);

private:
    void openReplayWindow(bool playImmediate);

    void testReplay();
};

