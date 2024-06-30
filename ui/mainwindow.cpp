#include "mainwindow.h"
#include <qgraphicsscene.h>
#include <qgraphicsitem.h>
#include <QVBoxLayout>
#include <QMenuBar>
#include <QFileDialog>
#include <QStatusBar>

#include "main_widget.h"
#include "change_tracker.h"
#include "action_manager.h"
#include "vehicle_manager.h"
#include "test/validation.h"

#include "spdlog/spdlog.h"


QGraphicsScene* g_scene;

MainWindow::MainWindow(QWidget* parent): QWidget(parent)
{
    setWindowTitle(tr("Road Runner"));
    setMinimumWidth(1600);
    setMinimumHeight(1200);

    QMenuBar* menu = new QMenuBar;
    QMenu* file = new QMenu("&File");
    auto newAction = file->addAction("New");
    auto loadAction = file->addAction("Open");
    auto saveAction = file->addAction("Save");
    menu->addMenu(file);

    QMenu* edit = new QMenu("&Edit");
    auto undoAction = edit->addAction("Undo");
    auto redoAction = edit->addAction("Redo");
    menu->addMenu(edit);

    QMenu* view = new QMenu("&Verify");
    auto verifyAction = view->addAction("Verify Now");
    auto alwaysVerifyAction = view->addAction("Always Verify");
    alwaysVerifyAction->setCheckable(true);
    alwaysVerifyAction->setChecked(RoadRunner::ChangeTracker::Instance()->VerifyUponChange);
    toggleSimAction = view->addAction("Toggle simulation");
    toggleSimAction->setCheckable(true);
    toggleSimAction->setChecked(false);
    menu->addMenu(view);

    QMenu* replay = new QMenu("&Replay");
    auto saveReplayAction = replay->addAction("Save");
    //auto animatedReplayAction = replay->addAction("Watch");
    auto debugReplayAction = replay->addAction("Debug");
    menu->addMenu(replay);

    scene = std::make_unique<QGraphicsScene>(this);
    g_scene = scene.get();
    vehicleManager = std::make_unique<VehicleManager>(this);
    
    mainWidget = std::make_unique<MainWidget>("Main View");
    mainWidget->view()->setScene(g_scene);

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
    connect(undoAction, &QAction::triggered, this, &MainWindow::undo);
    connect(redoAction, &QAction::triggered, this, &MainWindow::redo);
    connect(verifyAction, &QAction::triggered, this, &MainWindow::verifyMap);
    connect(alwaysVerifyAction, &QAction::triggered, this, &MainWindow::toggleAlwaysVerifyMap);
    connect(toggleSimAction, &QAction::triggered, this, &MainWindow::toggleSimulation);
    connect(saveReplayAction, &QAction::triggered, this, &MainWindow::saveActionHistory);
    connect(debugReplayAction, &QAction::triggered, this, &MainWindow::debugActionHistory);
    connect(mainWidget.get(), &MainWidget::HoveringChanged, this, &MainWindow::setHint);
    connect(mainWidget.get(), &MainWidget::FPSChanged, this, &MainWindow::setFPS);
    connect(mainWidget.get(), &MainWidget::InReadOnlyMode, this, &MainWindow::enableSimulation);

    srand(std::time(0));
}

MainWindow::~MainWindow() = default;

void MainWindow::newMap()
{
    mainWidget->Reset();
    RoadRunner::ChangeTracker::Instance()->Clear();
    RoadRunner::ActionManager::Instance()->Reset();
    assert(mainWidget->view()->scene()->items().isEmpty());
    // ROADRUNNERTODO: reset profile widget
}

void MainWindow::saveToFile()
{
    QString s = QFileDialog::getSaveFileName(
        this,
        "Choose save location",
        "C:\\Users\\guota\\Desktop\\RoadRunner",
        "OpenDrive (*.xodr)");
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
        "C:\\Users\\guota\\Desktop\\RoadRunner",
        "OpenDrive (*.xodr)");
    if (s.size() != 0)
    {
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
}

void MainWindow::toggleAlwaysVerifyMap(bool enable)
{
    RoadRunner::ChangeTracker::Instance()->VerifyUponChange = enable;
}

void MainWindow::saveActionHistory()
{
    QString s = QFileDialog::getSaveFileName(
        this,
        "Choose save location",
        "C:\\Users\\guota\\Desktop\\RoadRunner",
        "ActionHistory (*.dat)");
    if (s.size() != 0)
    {
        auto loc = s.toStdString();
        RoadRunner::ActionManager::Instance()->Save(loc);
    }
}

void MainWindow::debugActionHistory()
{
    QString s = QFileDialog::getOpenFileName(
        this,
        "Choose File to Open",
        "C:\\Users\\guota\\Desktop\\RoadRunner",
        "ActionHistory (*.dat)");
    if (s.size() != 0)
    {
        newMap();
        auto loc = s.toStdString();
        RoadRunner::ActionManager::Instance()->ReplayImmediate(loc);
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