#include "mainwindow.h"
#include <QVBoxLayout>
#include <QMenuBar>
#include <QFileDialog>
#include <QStatusBar>
#include <QApplication>
#include <QScreen>
#include <QDesktopWidget>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "main_widget.h"
#include "change_tracker.h"
#include "action_manager.h"
#include "vehicle_manager.h"
#include "test/validation.h"
#include "util.h"
#include "replay_window.h"
#include "preference.h"

#include "spdlog/spdlog.h"

#include "map_view_gl.h"

MainWindow* g_mainWindow;

extern UserPreference g_preference;

MainWindow::MainWindow(QWidget* parent): QWidget(parent)
{
    setWindowTitle(tr("Lane Maker"));
    setMinimumWidth(MinWidth);
    setMinimumHeight(MinHeight);
    resize(PreferredSize());

    g_mainWindow = this;

    QMenuBar* menu = new QMenuBar;
    QMenu* file = new QMenu("&File");
    auto newAction = file->addAction("New");
    auto loadAction = file->addAction("Open");
    auto saveAction = file->addAction("Save");
    auto preferenceAction = file->addAction("Preference");
    menu->addMenu(file);

    QMenu* edit = new QMenu("&Edit");
    auto undoAction = edit->addAction("Undo");
    auto redoAction = edit->addAction("Redo");
    auto verifyAction = edit->addAction("Verify Now");
    menu->addMenu(edit);

    QMenu* replay = new QMenu("&Replay");
    auto saveReplayAction = replay->addAction("Save");
    auto debugReplayAction = replay->addAction("Debug");
    auto controlledReplayAction = replay->addAction("Watch");
    menu->addMenu(replay);

    QMenu* simulation = new QMenu("&Simulation");
    toggleSimAction = simulation->addAction("Toggle simulation");
    toggleSimAction->setCheckable(true);
    toggleSimAction->setChecked(false);
    pauseResumeSimulation = simulation->addAction("Paused");
    pauseResumeSimulation->setCheckable(true);
    pauseResumeSimulation->setChecked(false);
    pauseResumeSimulation->setEnabled(false);
    menu->addMenu(simulation);

#ifdef __linux__
    // In linux, this dialog completely blocks main mainwindow if spawned as child
    replayWindow = std::make_unique<ReplayWindow>();
#else
    replayWindow = std::make_unique<ReplayWindow>(this);
#endif
    preferenceWindow = std::make_unique<PreferenceWindow>(this);

    vehicleManager = std::make_unique<VehicleManager>(this);
    
    mainWidget = std::make_unique<MainWidget>();
    mainWidget->toggleAntialiasing(g_preference.antiAlias);

    auto mainLayout = new QVBoxLayout;
    mainLayout->addWidget(menu);
    mainLayout->addWidget(mainWidget.get());

    auto bottomLayout = new QHBoxLayout;
    hintStatus = std::make_unique<QStatusBar>();
    bottomLayout->addWidget(hintStatus.get());
    fpsStatus = std::make_unique<QStatusBar>();
    bottomLayout->addStretch();
    bottomLayout->addWidget(fpsStatus.get());
    mainLayout->addLayout(bottomLayout);
    
    setLayout(mainLayout);

    connect(newAction, &QAction::triggered, this, &MainWindow::newMap);
    connect(saveAction, &QAction::triggered, this, &MainWindow::saveToFile);
    connect(loadAction, &QAction::triggered, this, &MainWindow::loadFromFile);
    connect(preferenceAction, &QAction::triggered, preferenceWindow.get(), &PreferenceWindow::open);
    connect(undoAction, &QAction::triggered, this, &MainWindow::undo);
    connect(redoAction, &QAction::triggered, this, &MainWindow::redo);
    connect(verifyAction, &QAction::triggered, this, &MainWindow::verifyMap);
    connect(toggleSimAction, &QAction::toggled, this, &MainWindow::toggleSimulation);
    connect(toggleSimAction, &QAction::toggled, this, [=](bool enabled) {
        undoAction->setEnabled(!enabled); redoAction->setEnabled(!enabled); });
    connect(pauseResumeSimulation, &QAction::toggled, vehicleManager.get(), &VehicleManager::TogglePause);
    connect(saveReplayAction, &QAction::triggered, this, &MainWindow::saveActionHistory);
    connect(debugReplayAction, &QAction::triggered, this, &MainWindow::debugActionHistory);
    connect(controlledReplayAction, &QAction::triggered, this, &MainWindow::playActionHistory);
    connect(replayWindow.get(), &ReplayWindow::Restart, this, &MainWindow::reset);
    connect(mainWidget.get(), &MainWidget::FPSChanged, this, &MainWindow::setFPS);
    connect(preferenceWindow.get(), &PreferenceWindow::ToggleAA, mainWidget.get(), &MainWidget::toggleAntialiasing);

    connect(mainWidget->mapViewGL, &LM::MapViewGL::MousePerformedAction, this, &MainWindow::updateHint);

    if (g_preference.showWelcome)
        preferenceWindow->open();
    srand(std::time(0)); // traffic simulation
}

MainWindow::~MainWindow() = default;

QSize MainWindow::PreferredSize() const
{
    auto available = QApplication::desktop()->screenGeometry();
    int preferredHeight = available.height() * 0.6;
    int preferredWidth = available.width() * 0.5;
    return QSize(std::max(preferredWidth, MinWidth), std::max(preferredHeight, MinHeight));
}

void MainWindow::resizeEvent(QResizeEvent* e)
{
    QWidget::resizeEvent(e);
    if (recordResize)
        LM::ActionManager::Instance()->Record(e->oldSize(), e->size());
}

void MainWindow::newMap()
{
    auto oldsize = size();
    reset();
    LM::ActionManager::Instance()->Record(oldsize, size());
}

void MainWindow::reset()
{
    auto prevLevel = spdlog::get_level();
    /*Road destruction order be random, which could cause temporary invalid state.*/
    //spdlog::set_level(spdlog::level::critical);

    stopSimulation();
    mainWidget->Reset();
    LM::ChangeTracker::Instance()->Clear();
    LM::ActionManager::Instance()->Reset();
    LM::g_mapViewGL->ResetCamera();
    LM::g_createRoadElevationOption = 0;
    resizeDontRecord(PreferredSize().width(), PreferredSize().height());
    loadedFileName.clear();
    LM::g_mapViewGL->renderNow();
    //spdlog::set_level(prevLevel);
}

void MainWindow::resizeDontRecord(int w, int h)
{
    recordResize = false;
    resize(w, h);
    recordResize = true;
}

void MainWindow::saveToFile()
{
    auto saveLoc = loadedFileName.empty() ? LM::DefaultSaveFolder().string() : loadedFileName;
    QString s = QFileDialog::getSaveFileName(
        this,
        "Choose save location",
        saveLoc.c_str(),
        "OpenDrive (*.xodr)", nullptr
#ifdef __linux__
        ,QFileDialog::DontUseNativeDialog
#endif
        );
    if (s.size() != 0)
    {
        auto loc = s.toStdString();
        LM::ChangeTracker::Instance()->Save(loc);
        if (loadedFileName.empty())
        {
            loadedFileName = loc;
        }
    }
}

void MainWindow::loadFromFile()
{
    QString s = QFileDialog::getOpenFileName(
        this, 
        "Choose File to Open",
        LM::DefaultSaveFolder().string().c_str(),
        "OpenDrive (*.xodr)", nullptr
#ifdef __linux__
        ,QFileDialog::DontUseNativeDialog
#endif
    );
    if (s.size() != 0)
    {
        reset();
        loadedFileName = s.toStdString();

        bool supported = LM::ChangeTracker::Instance()->Load(loadedFileName);
        if (!supported)
        {
            spdlog::error("xodr map needs to contain custom LaneProfile!");
        }
        std::ifstream ifs(loadedFileName);
        std::stringstream buffer;
        buffer << ifs.rdbuf();
        LM::ActionManager::Instance()->Record(buffer.str());

        LM::g_mapViewGL->renderNow();
    }
}

void MainWindow::undo()
{
    LM::ActionManager::Instance()->Record(LM::ActionType::Action_Undo);
    if (!LM::ChangeTracker::Instance()->Undo())
    {
        spdlog::warn("Cannot undo");
    }
    else
    {
        LM::g_mapViewGL->renderNow();
    }
}

void MainWindow::redo()
{
    LM::ActionManager::Instance()->Record(LM::ActionType::Action_Redo);
    if (!LM::ChangeTracker::Instance()->Redo())
    {
        spdlog::warn("Cannot redo");
    }
    else
    {
        LM::g_mapViewGL->renderNow();
    }
}

void MainWindow::verifyMap()
{
    LTest::Validation::ValidateMap();
    spdlog::info("Done map verification.");
}

void MainWindow::saveActionHistory()
{
    QString s = QFileDialog::getSaveFileName(
        this,
        "Choose save location",
        LM::DefaultSaveFolder().string().c_str(),
        "ActionHistory (*.dat)", nullptr
#ifdef __linux__
        ,QFileDialog::DontUseNativeDialog
#endif
        );
    if (s.size() != 0)
    {
        auto loc = s.toStdString();
        LM::ActionManager::Instance()->Save(loc);
    }
}

void MainWindow::debugActionHistory()
{
    openReplayWindow(true);
}

void MainWindow::playActionHistory()
{
    openReplayWindow(false);
}

void MainWindow::openReplayWindow(bool playImmediate)
{
    QString s = QFileDialog::getOpenFileName(
        this,
        "Choose File to Open",
        LM::DefaultSaveFolder().string().c_str(),
        "ActionHistory (*.dat)", nullptr
#ifdef __linux__
        ,QFileDialog::DontUseNativeDialog
#endif
        );
    if (!s.isEmpty())
    {
        reset();
        QScreen* screen = QGuiApplication::primaryScreen();
        QRect  screenGeometry = screen->geometry();
        replayWindow->LoadHistory(s.toStdString(), playImmediate);
        const int replayWindowWidth = 300;
        replayWindow->setGeometry(
            std::min(screenGeometry.width() - replayWindowWidth, geometry().right()), geometry().top(), 
            replayWindowWidth, geometry().height());
        replayWindow->open();
    }
}

void MainWindow::toggleSimulation(bool enable)
{
    pauseResumeSimulation->setChecked(false);
    if (enable)
    {
        vehicleManager->Begin();
    }
    else
    {
        vehicleManager->End();
    }
    pauseResumeSimulation->setEnabled(enable);
    mainWidget->GoToSimulationMode(enable);
}

void MainWindow::stopSimulation()
{
    if (toggleSimAction->isChecked())
    {
        vehicleManager->End();
        toggleSimAction->setChecked(false);
    }
}

void MainWindow::setFPS(QString msg)
{
    fpsStatus->showMessage(msg);
}

void MainWindow::updateHint()
{
    auto groundInfo = QString("(%1, %2) ")
        .arg(LM::g_PointerOnGround[0])
        .arg(LM::g_PointerOnGround[1]);
    auto roadInfo = LM::g_PointerRoadID.empty() ?
        QString("VBuffer: %1%")
        .arg(mainWidget->mapViewGL->VBufferUseage_pct()) :
        QString("Road %1 @%2 Lane %3")
        .arg(LM::g_PointerRoadID.c_str())
        .arg(LM::g_PointerRoadS, 6, 'f', 3)
        .arg(LM::g_PointerLane);
    groundInfo.append(roadInfo);
    if (LM::g_PointerVehicle != -1)
    {
        groundInfo.append(QString("  Vehicle: %1").arg(LM::g_PointerVehicle));
    }

    hintStatus->showMessage(groundInfo);
}

void MainWindow::closeEvent(QCloseEvent* e)
{
    testReplay();
    reset();
    mainWidget->mapViewGL->CleanupResources();
    QWidget::closeEvent(e);
}

#ifdef __linux__
void MainWindow::keyPressEvent(QKeyEvent* e)
{
    mainWidget->mapViewGL->keyPressEvent(e);
}
#endif

void MainWindow::testReplay()
{
    auto recordPath = LM::ActionManager::Instance()->AutosavePath();
    if (g_preference.alwaysVerify
        && std::filesystem::exists(recordPath))
    {
        g_preference.alwaysVerify = false; // No verification during replay
        auto saveFolder = LM::DefaultSaveFolder();
        auto originalPath = saveFolder / (std::string("compare_a_") + LM::RunTimestamp() + std::string(".xodr"));
        auto originalPathStr = originalPath.string();
        LM::ChangeTracker::Instance()->Save(originalPathStr);

        reset();

        quitReplayComplete = false;
        connect(replayWindow.get(), &ReplayWindow::DoneReplay, this, &MainWindow::onReplayDone);
        replayWindow->LoadHistory(recordPath, true);
        replayWindow->exec();

        if (quitReplayComplete)
        {
            auto replayPath = saveFolder / (std::string("compare_b_") + LM::RunTimestamp() + std::string(".xodr"));
            auto replayPathStr = replayPath.string();
            LM::ChangeTracker::Instance()->Save(replayPathStr);

            if (!LTest::Validation::CompareFiles(originalPathStr, replayPathStr))
            {
                LM::ActionManager::Instance()->MarkException();
                spdlog::error("Replay result is different from original map! Check {} for details.", recordPath);
            }
            else
            {
                // On success, clean up temporary saves
                std::remove(originalPathStr.c_str());
                std::remove(replayPathStr.c_str());
                spdlog::info("Action replay test: OK");
            }
        }
        else
        {
            // cancelled by user
            std::remove(originalPathStr.c_str());
            spdlog::info("Action replay test: Cancelled");
        }
    }
    
    if (LM::ActionManager::Instance()->CleanAutoSave())
    {
        std::remove(recordPath.c_str());
    }
}

void MainWindow::onReplayDone(bool completed)
{
    quitReplayComplete = completed;
    replayWindow->close();
}

void MainWindow::runReplay(std::string replay)
{
    if (!std::filesystem::exists(replay))
    {
        spdlog::warn("Unable to run speficied replay(s): File does not exist");
        return;
    }
    // Always run verify during replay
    g_preference.alwaysVerify = true;
    connect(replayWindow.get(), &ReplayWindow::DoneReplay, this, &MainWindow::onReplayDone);

    if (std::filesystem::is_regular_file(replay) && std::filesystem::path(replay).extension() == ".dat")
    {
        spdlog::info(">> Running {}", replay);
        replayWindow->LoadHistory(replay, true);
        replayWindow->exec();
    }
    else if (std::filesystem::is_directory(replay))
    {
        for (std::filesystem::recursive_directory_iterator i(replay), end; i != end; ++i)
        {
            auto path = i->path();
            if (std::filesystem::is_regular_file(path) && std::filesystem::path(path).extension() == ".dat")
            {
                spdlog::info(">> Running {}", path.string());
                replayWindow->LoadHistory(path.string(), true);
                replayWindow->exec();
                reset();
            }
        }
    }
}
