#include "mainwindow.h"
#include <qgraphicsscene.h>
#include <qgraphicsitem.h>
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

#include <QPushButton>
#include <QGraphicsProxyWidget>


QGraphicsScene* g_scene;

MainWindow* g_mainWindow;

extern UserPreference g_preference;

MainWindow::MainWindow(QWidget* parent): QWidget(parent)
{
    setWindowTitle(tr("Road Runner"));
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
    toggleSimAction = edit->addAction("Toggle simulation");
    toggleSimAction->setCheckable(true);
    toggleSimAction->setChecked(false);
    menu->addMenu(edit);

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

    // Overlay button example 
    /*
    QPushButton* testButton = new QPushButton("Test button");
    QGraphicsProxyWidget* proxy = scene->addWidget(testButton);
    proxy->setGeometry(QRectF(100, 100, 200, 300));
    QTransform matrix;
    matrix.scale(1, -1);
    proxy->setTransform(matrix);
    */

    g_scene = scene.get();
    vehicleManager = std::make_unique<VehicleManager>(this);
    
    mainWidget = std::make_unique<MainWidget>(g_scene);
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
    connect(replayWindow.get(), &ReplayWindow::Restart, this, &MainWindow::reset);
    connect(mainWidget.get(), &MainWidget::HoveringChanged, this, &MainWindow::setHint);
    connect(mainWidget.get(), &MainWidget::FPSChanged, this, &MainWindow::setFPS);
    connect(mainWidget.get(), &MainWidget::InReadOnlyMode, this, &MainWindow::enableSimulation);
    connect(preferenceWindow.get(), &PreferenceWindow::ToggleAA, mainWidget.get(), &MainWidget::toggleAntialiasing);
    //connect(testButton, &QPushButton::clicked, []() {spdlog::info("Test btn press"); });

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
        RoadRunner::ActionManager::Instance()->Record(e->oldSize(), e->size());
}

void MainWindow::newMap()
{
    auto oldsize = size();
    reset();
    RoadRunner::ActionManager::Instance()->Record(oldsize, size());
}

void MainWindow::reset()
{
    auto prevLevel = spdlog::get_level();
    /*Road destruction order be random, which could cause temporary invalid state.*/
    spdlog::set_level(spdlog::level::critical);

    mainWidget->Reset();
    RoadRunner::ChangeTracker::Instance()->Clear();
    RoadRunner::ActionManager::Instance()->Reset();
    assert(mainWidget->view()->scene()->items().isEmpty());
    resizeDontRecord(PreferredSize().width(), PreferredSize().height());

    spdlog::set_level(prevLevel);
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
        reset();
        auto loc = s.toStdString();

        bool supported = RoadRunner::ChangeTracker::Instance()->Load(loc);
        if (!supported)
        {
            spdlog::error("xodr map needs to contain custom LaneProfile!");
        }
        std::ifstream ifs(loc);
        std::stringstream buffer;
        buffer << ifs.rdbuf();
        RoadRunner::ActionManager::Instance()->Record(buffer.str());

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

void MainWindow::keyPressEvent(QKeyEvent* e)
{
    if (e->key() == Qt::Key_F1)
    {
        preferenceWindow->open();
    }
    QWidget::keyPressEvent(e);
}

void MainWindow::testReplay()
{
    if (g_preference.alwaysVerify
        && std::filesystem::exists(RoadRunner::ActionManager::Instance()->AutosavePath()))
    {
        g_preference.alwaysVerify = false; // No verification during replay
        auto saveFolder = RoadRunner::DefaultSaveFolder();
        auto originalPath = saveFolder / (std::string("compare_a_") + RoadRunner::RunTimestamp() + std::string(".xodr"));
        auto originalPathStr = originalPath.string();
        RoadRunner::ChangeTracker::Instance()->Save(originalPathStr);

        reset();

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
