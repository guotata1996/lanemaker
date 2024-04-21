#include "mainwindow.h"
#include <qgraphicsscene.h>
#include <qgraphicsitem.h>
#include <QVBoxLayout>
#include <QMenuBar>
#include <QFileDialog>

#include "view.h"
#include "change_tracker.h"

#include "spdlog/spdlog.h"


QGraphicsScene* g_scene;

MainWindow::MainWindow(QWidget* parent): QWidget(parent)
{
    setWindowTitle(tr("Road Runner"));
    setMinimumWidth(1600);
    setMinimumHeight(900);

    QMenuBar* menu = new QMenuBar;
    QMenu* file = new QMenu("&File");
    auto loadAction = file->addAction("Open");
    auto saveAction = file->addAction("Save");
    menu->addMenu(file);

    QMenu* edit = new QMenu("&Edit");
    auto undoAction = edit->addAction("Undo");
    auto redoAction = edit->addAction("Redo");
    menu->addMenu(edit);

    scene = std::make_unique<QGraphicsScene>(this);
    g_scene = scene.get();
    
    auto view = new MainWidget("Main View");
    view->view()->setScene(g_scene);

    auto mainLayout = new QVBoxLayout;
    mainLayout->addWidget(menu);
    mainLayout->addWidget(view);
    
    setLayout(mainLayout);

    connect(saveAction, &QAction::triggered, this, &MainWindow::saveToFile);
    connect(loadAction, &QAction::triggered, this, &MainWindow::loadFromFile);
    connect(undoAction, &QAction::triggered, this, &MainWindow::undo);
    connect(redoAction, &QAction::triggered, this, &MainWindow::redo);
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
    }
}

void MainWindow::undo()
{
    if (!RoadRunner::ChangeTracker::Instance()->Undo())
    {
        spdlog::warn("Cannot undo");
    }
}

void MainWindow::redo()
{
    if (!RoadRunner::ChangeTracker::Instance()->Redo())
    {
        spdlog::warn("Cannot redo");
    }
}