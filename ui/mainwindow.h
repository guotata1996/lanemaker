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

namespace LM
{
    class MapViewGL;
}

class MainWindow : public QWidget
{
public:
    MainWindow(QWidget* parent = nullptr);

    ~MainWindow();

    void resizeDontRecord(int w, int h);

    void runReplay(std::string replay);

protected:
    void resizeEvent(QResizeEvent*) override;

    void closeEvent(QCloseEvent* event) override;

#ifdef __linux__
    void keyPressEvent(QKeyEvent*) override;
#endif

private:
    const int MinWidth = 640;
    const int MinHeight = 480;

    QSize PreferredSize() const;

    std::unique_ptr<QStatusBar> hintStatus;
    std::unique_ptr<QStatusBar> fpsStatus;

    std::unique_ptr<MainWidget> mainWidget;
    std::unique_ptr<VehicleManager> vehicleManager;

    std::unique_ptr<ReplayWindow> replayWindow;
    std::unique_ptr<PreferenceWindow> preferenceWindow;

    QAction* toggleSimAction;
    QAction* pauseResumeSimulation;

    bool quitReplayComplete;

    bool recordResize = true;

    std::string loadedFileName;

public slots:
    void ReplaySingleStep();

    void undo();

    void redo();

private slots:
    void updateHint();
    void setFPS(QString);

    void newMap();

    void saveToFile();

    void loadFromFile();

    void verifyMap();

    void saveActionHistory();

    void debugActionHistory();

    void playActionHistory();

    void toggleSimulation(bool);

    void stopSimulation();

    void onReplayDone(bool);

private:
    void openReplayWindow(bool playImmediate);

    void testReplay();

    void reset();
};

