#include "mainwindow.h"
#include <qgraphicsscene.h>
#include <qgraphicsitem.h>
#include <QVBoxLayout>
#include <QMenuBar>
#include <QFileDialog>

#include "view.h"
#include "change_tracker.h"
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
    auto loadAction = file->addAction("Open");
    auto saveAction = file->addAction("Save");
    menu->addMenu(file);

    QMenu* edit = new QMenu("&Edit");
    auto undoAction = edit->addAction("Undo");
    auto redoAction = edit->addAction("Redo");
    menu->addMenu(edit);

    QMenu* view = new QMenu("&View");
    auto verifyAction = view->addAction("Verify Now");
    auto alwaysVerifyAction = view->addAction("Always Verify");
    alwaysVerifyAction->setCheckable(true);
    alwaysVerifyAction->setChecked(RoadRunner::ChangeTracker::Instance()->VerifyUponChange);
    menu->addMenu(view);

    scene = std::make_unique<QGraphicsScene>(this);
    g_scene = scene.get();
    
    mainWidget = new MainWidget("Main View");
    mainWidget->view()->setScene(g_scene);

    auto mainLayout = new QVBoxLayout;
    mainLayout->addWidget(menu);
    mainLayout->addWidget(mainWidget);
    
    setLayout(mainLayout);

    connect(saveAction, &QAction::triggered, this, &MainWindow::saveToFile);
    connect(loadAction, &QAction::triggered, this, &MainWindow::loadFromFile);
    connect(undoAction, &QAction::triggered, this, &MainWindow::undo);
    connect(redoAction, &QAction::triggered, this, &MainWindow::redo);
    connect(verifyAction, &QAction::triggered, this, &MainWindow::verifyMap);
    connect(alwaysVerifyAction, &QAction::triggered, this, &MainWindow::toggleAlwaysVerifyMap);
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