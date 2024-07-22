#include "mainwindow.h"
#include <qgraphicsscene.h>
#include <qgraphicsitem.h>
#include <QVBoxLayout>
#include <QMenuBar>
#include <QFileDialog>
#include <QStatusBar>
#include <QApplication>
#include <QScreen>
#include <filesystem>

#include "main_widget.h"
#include "change_tracker.h"
#include "action_manager.h"
#include "vehicle_manager.h"
#include "test/validation.h"
#include "util.h"
#include "replay_window.h"
#include "preference.h"

#include "spdlog/spdlog.h"


QGraphicsScene* g_scene;

MainWindow* g_mainWindow;

extern UserPreference g_preference;

MainWindow::MainWindow(QWidget* parent): QWidget(parent)
{
    setWindowTitle(tr("Road Runner"));
    setMinimumWidth(StartWidth);
    setMinimumHeight(StartHeight);

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
    menu->addMenu(edit);

    QMenu* view = new QMenu("&Verify");
    auto verifyAction = view->addAction("Verify Now");
    toggleSimAction = view->addAction("Toggle simulation");
    toggleSimAction->setCheckable(true);
    toggleSimAction->setChecked(false);
    menu->addMenu(view);

    QMenu* replay = new QMenu("&Replay");
    auto saveReplayAction = replay->addAction("Save");
    auto debugReplayAction = replay->addAction("Debug");
    auto controlledReplayAction = replay->addAction("Watch");
    menu->addMenu(replay);
#ifdef __linux__
    // In linux, this dialog completely blocks main mainwindow if spawned as child
    replayWindow = std::make_unique<ReplayWindow>();
#else
    replayWindow = std::make_unique<ReplayWindow>(this);
#endif
    preferenceWindow = std::make_unique<PreferenceWindow>(this);

    scene = std::make_unique<QGraphicsScene>(this);
    g_scene = scene.get();
    vehicleManager = std::make_unique<VehicleManager>(this);
    
    mainWidget = std::make_unique<MainWidget>("Main View");
    mainWidget->view()->setScene(g_scene);
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
    connect(toggleSimAction, &QAction::triggered, this, &MainWindow::toggleSimulation);
    connect(saveReplayAction, &QAction::triggered, this, &MainWindow::saveActionHistory);
    connect(debugReplayAction, &QAction::triggered, this, &MainWindow::debugActionHistory);
    connect(controlledReplayAction, &QAction::triggered, this, &MainWindow::playActionHistory);
    connect(replayWindow.get(), &ReplayWindow::Restart, this, &MainWindow::newMap);
    connect(mainWidget.get(), &MainWidget::HoveringChanged, this, &MainWindow::setHint);
    connect(mainWidget.get(), &MainWidget::FPSChanged, this, &MainWindow::setFPS);
    connect(mainWidget.get(), &MainWidget::InReadOnlyMode, this, &MainWindow::enableSimulation);
    connect(preferenceWindow.get(), &PreferenceWindow::ToggleAA, mainWidget.get(), &MainWidget::toggleAntialiasing);

    srand(std::time(0));
}

MainWindow::~MainWindow() = default;

void MainWindow::resizeEvent(QResizeEvent* e)
{
    QWidget::resizeEvent(e);
    if (recordResize)
        RoadRunner::ActionManager::Instance()->Record(e->oldSize(), e->size());
}

void MainWindow::newMap()
{
    mainWidget->Reset();
    RoadRunner::ChangeTracker::Instance()->Clear();
    RoadRunner::ActionManager::Instance()->Reset();
    assert(mainWidget->view()->scene()->items().isEmpty());
    resizeDontRecord(StartWidth, StartHeight);
}

void MainWindow::resizeDontRecord(int w, int h)
{
    recordResize = false;
    resize(w, h);
    recordResize = true;
}

void MainWindow::saveToFile()
{
    QString s = QFileDialog::getSaveFileName(
        this,
        "Choose save location",
        RoadRunner::DefaultSaveFolder().string().c_str(),
        "OpenDrive (*.xodr)", nullptr
#ifdef __linux__
        ,QFileDialog::DontUseNativeDialog
#endif
        );
    if (s.size() != 0)
    {
        auto loc = s.toStdString();
        RoadRunner::ChangeTracker::Instance()->Save(loc);
    }
}

void MainWindow::loadFromFile()
{
    QString s = QFileDialog::getOpenFileName(
        this, 
        "Choose File to Open",
        RoadRunner::DefaultSaveFolder().string().c_str(),
        "OpenDrive (*.xodr)", nullptr
#ifdef __linux__
        ,QFileDialog::DontUseNativeDialog
#endif
    );
    if (s.size() != 0)
    {
        newMap();
        auto loc = s.toStdString();
        bool supported = RoadRunner::ChangeTracker::Instance()->Load(loc);
        if (!supported)
        {
            spdlog::error("xodr map needs to contain custom RoadProfile!");
        }
        mainWidget->AdjustSceneRect();
    }
}

void MainWindow::undo()
{
    RoadRunner::ActionManager::Instance()->Record(RoadRunner::ActionType::Action_Undo);
    if (!RoadRunner::ChangeTracker::Instance()->Undo())
    {
        spdlog::warn("Cannot undo");
    }
    else
    {
        mainWidget->AdjustSceneRect();
    }
}

void MainWindow::redo()
{
    RoadRunner::ActionManager::Instance()->Record(RoadRunner::ActionType::Action_Redo);
    if (!RoadRunner::ChangeTracker::Instance()->Redo())
    {
        spdlog::warn("Cannot redo");
    }
    else
    {
        mainWidget->AdjustSceneRect();
    }
}

void MainWindow::verifyMap()
{
    RoadRunnerTest::Validation::ValidateMap();
    spdlog::info("Done map verification.");
}

void MainWindow::saveActionHistory()
{
    if (!RoadRunner::ActionManager::Instance()->Replayable())
    {
        spdlog::warn("Abort: can't save unreplayable history!");
        return;
    }

    QString s = QFileDialog::getSaveFileName(
        this,
        "Choose save location",
        RoadRunner::DefaultSaveFolder().string().c_str(),
        "ActionHistory (*.dat)", nullptr
#ifdef __linux__
        ,QFileDialog::DontUseNativeDialog
#endif
        );
    if (s.size() != 0)
    {
        auto loc = s.toStdString();
        RoadRunner::ActionManager::Instance()->Save(loc);
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
        RoadRunner::DefaultSaveFolder().string().c_str(),
        "ActionHistory (*.dat)", nullptr
#ifdef __linux__
        ,QFileDialog::DontUseNativeDialog
#endif
        );
    if (!s.isEmpty())
    {
        newMap();
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
    if (enable)
    {
        vehicleManager->Begin();
    }
    else
    {
        vehicleManager->End();
    }
}

void MainWindow::enableSimulation(bool available)
{
    toggleSimAction->setEnabled(available);
    if (toggleSimAction->isChecked() && !available)
    {
        vehicleManager->End();
        toggleSimAction->setChecked(false);
    }
}

void MainWindow::setHint(QString msg) 
{
    hintStatus->showMessage(msg);
}

void MainWindow::setFPS(QString msg)
{
    fpsStatus->showMessage(msg);
}

void MainWindow::closeEvent(QCloseEvent* e)
{
    vehicleManager->End();
    testReplay();
    QWidget::closeEvent(e);
}

void MainWindow::testReplay()
{
    if (g_preference.alwaysVerify
        && RoadRunner::ActionManager::Instance()->Replayable()
        && std::filesystem::exists(RoadRunner::ActionManager::Instance()->AutosavePath()))
    {
        g_preference.alwaysVerify = false; // No verification during replay
        auto saveFolder = RoadRunner::DefaultSaveFolder();
        auto originalPath = saveFolder / (std::string("compare_a_") + RoadRunner::RunTimestamp() + std::string(".xodr"));
        auto originalPathStr = originalPath.string();
        RoadRunner::ChangeTracker::Instance()->Save(originalPathStr);

        newMap();

        quitReplayComplete = false;
        connect(replayWindow.get(), &ReplayWindow::DoneReplay, this, &MainWindow::onReplayDone);
        replayWindow->LoadHistory(RoadRunner::ActionManager::Instance()->AutosavePath(), true);
        replayWindow->exec();

        if (quitReplayComplete)
        {
            auto replayPath = saveFolder / (std::string("compare_b_") + RoadRunner::RunTimestamp() + std::string(".xodr"));
            auto replayPathStr = replayPath.string();
            RoadRunner::ChangeTracker::Instance()->Save(replayPathStr);

            if (!RoadRunnerTest::Validation::CompareFiles(originalPathStr, replayPathStr))
            {
                RoadRunner::ActionManager::Instance()->MarkException();
                spdlog::error("Replay result is different from original map! Check {} for details.",
                    RoadRunner::ActionManager::Instance()->AutosavePath());
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
    
    if (RoadRunner::ActionManager::Instance()->CleanAutoSave())
    {
        std::remove(RoadRunner::ActionManager::Instance()->AutosavePath().c_str());
    }
}

void MainWindow::onReplayDone(bool completed)
{
    quitReplayComplete = completed;
    replayWindow->close();
}
