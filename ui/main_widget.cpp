// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include <QtWidgets>

#include "main_widget.h"
#include "map_view_gl.h"
#include "main_window.h"
#include "action_manager.h"
#include "road_drawing.h"
#include "change_tracker.h"
#include "LaneConfigWidget.h"
#include "DrawOptionDialog.h"

MainWidget* MainWidget::instance = nullptr;

MainWidget::MainWidget(QWidget* parent)
    : QFrame(parent), laneConfig(new LaneConfigWidget), 
    drawOptionDialog(new DrawOptionDialog(g_mainWindow))
{
    instance = this;
    setFrameStyle(Sunken | StyledPanel);

    QSurfaceFormat format;
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setVersion(3, 3);
    format.setDepthBufferSize(16);

    mapViewGL = new LM::MapViewGL;
    mapViewGL->setFormat(format);
    mapViewGL->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    mapViewGL->setFocusPolicy(Qt::ClickFocus);
    mapViewGL->setMouseTracking(true);

    int size = style()->pixelMetric(QStyle::PM_ToolBarIconSize);
    QSize iconSize(size, size);

    // Label layout
    QSize largeIconSize = iconSize * 1.75;
    QHBoxLayout* labelLayout = new QHBoxLayout;
    createModeButton = new QToolButton;
    createModeButton->setToolTip(tr("Road Mode"));
    createModeButton->setIcon(QPixmap(":/icons/road_mode.png"));
    createModeButton->setIconSize(largeIconSize);
    createModeButton->setCheckable(true);
    createModeButton->setChecked(false);
    createLaneModeButton = new QToolButton;
    createLaneModeButton->setToolTip(tr("Lane Mode"));
    createLaneModeButton->setIcon(QPixmap(":/icons/lane_mode.png"));
    createLaneModeButton->setIconSize(largeIconSize);
    createLaneModeButton->setCheckable(true);
    createLaneModeButton->setChecked(false);
    destroyModeButton = new QToolButton;
    destroyModeButton->setToolTip(tr("Destroy Mode"));
    destroyModeButton->setIcon(QPixmap(":/icons/destroy_mode.png"));
    destroyModeButton->setIconSize(largeIconSize);
    destroyModeButton->setCheckable(true);
    destroyModeButton->setChecked(false);
    modifyModeButton = new QToolButton;
    modifyModeButton->setToolTip(tr("Modify Mode"));
    modifyModeButton->setIcon(QPixmap(":/icons/modify_mode.PNG"));
    modifyModeButton->setIconSize(largeIconSize);
    modifyModeButton->setCheckable(true);
    modifyModeButton->setChecked(false);
    dragModeButton = new QToolButton;
    dragModeButton->setToolTip(tr("Drag Mode"));
    dragModeButton->setIcon(QPixmap(":/icons/view_mode.png"));
    dragModeButton->setIconSize(largeIconSize);
    dragModeButton->setCheckable(true);
    dragModeButton->setChecked(false);

    pointerModeGroup = new QButtonGroup(this);
    pointerModeGroup->setExclusive(true);
    pointerModeGroup->addButton(createModeButton);
    pointerModeGroup->addButton(createLaneModeButton);
    pointerModeGroup->addButton(modifyModeButton);
    pointerModeGroup->addButton(destroyModeButton);
    pointerModeGroup->addButton(dragModeButton);

    labelLayout->addWidget(laneConfig);
    QSizePolicy sp_retain = laneConfig->sizePolicy();
    sp_retain.setRetainSizeWhenHidden(true);
    laneConfig->setSizePolicy(sp_retain);
    laneConfig->hide();
    g_laneConfig = laneConfig;
    labelLayout->addSpacing(2 * size);
    labelLayout->addWidget(createModeButton);
    labelLayout->addWidget(createLaneModeButton);
    labelLayout->addWidget(modifyModeButton);
    labelLayout->addWidget(destroyModeButton);
    labelLayout->addWidget(dragModeButton);
    labelLayout->addStretch();

    auto loadButton = new QToolButton(this);
    loadButton->setIcon(QPixmap(":/icons/open.png"));
    loadButton->setIconSize(largeIconSize);
    labelLayout->addWidget(loadButton);
    auto saveButton = new QToolButton(this);
    saveButton->setIcon(QPixmap(":/icons/save.png"));
    saveButton->setIconSize(largeIconSize);
    labelLayout->addWidget(saveButton);

    labelLayout->addSpacing(size);

    auto undoButton = new QToolButton(this);
    undoButton->setIcon(QPixmap(":/icons/undo.png"));
    undoButton->setIconSize(largeIconSize);
    labelLayout->addWidget(undoButton);

    auto redoButton = new QToolButton(this);
    redoButton->setIcon(QPixmap(":/icons/redo.png"));
    redoButton->setIconSize(largeIconSize);
    labelLayout->addWidget(redoButton);

    labelLayout->addSpacing(size);

    auto drawOptionButton = new QToolButton(this);
    drawOptionButton->setIcon(QPixmap(":/icons/draw_option.png"));
    drawOptionButton->setIconSize(largeIconSize);
    labelLayout->addWidget(drawOptionButton);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addLayout(labelLayout);
    mainLayout->addWidget(mapViewGL);
    setLayout(mainLayout);

    connect(createModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoCreateRoadMode);
    connect(createLaneModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoCreateLaneMode);
    connect(destroyModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoDestroyMode);
    connect(modifyModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoModifyMode);
    connect(dragModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoDragMode);
    connect(mapViewGL, &LM::MapViewGL::MousePerformedAction, this, &MainWidget::OnMouseAction);
    connect(mapViewGL, &LM::MapViewGL::KeyPerformedAction, this, &MainWidget::OnKeyPress);

    connect(saveButton, &QAbstractButton::clicked, g_mainWindow, &MainWindow::saveToFile);
    connect(loadButton, &QAbstractButton::clicked, g_mainWindow, &MainWindow::loadFromFile);
    connect(undoButton, &QAbstractButton::clicked, g_mainWindow, &MainWindow::undo);
    connect(redoButton, &QAbstractButton::clicked, g_mainWindow, &MainWindow::redo);
    connect(drawOptionButton, &QAbstractButton::clicked, drawOptionDialog, &QDialog::open);
    Reset();
}

MainWidget* MainWidget::Instance()
{
    return instance;
}

void MainWidget::gotoCreateRoadMode(bool checked)
{
    if (!checked) return;
    SetEditMode(LM::Mode_Create);
    LM::ActionManager::Instance()->Record(LM::Mode_Create);
    laneConfig->GotoRoadMode();
}

void MainWidget::gotoCreateLaneMode(bool checked)
{
    if (!checked) return;
    SetEditMode(LM::Mode_CreateLanes);
    LM::ActionManager::Instance()->Record(LM::Mode_CreateLanes);
    laneConfig->GotoLaneMode();
}

void MainWidget::gotoDestroyMode(bool checked)
{
    if (!checked) return;
    SetEditMode(LM::Mode_Destroy);
    LM::ActionManager::Instance()->Record(LM::Mode_Destroy);
    laneConfig->hide();
}

void MainWidget::gotoModifyMode(bool checked)
{
    if (!checked) return;
    SetEditMode(LM::Mode_Modify);
    LM::ActionManager::Instance()->Record(LM::Mode_Modify);
    laneConfig->GotoRoadMode();
}

void MainWidget::gotoDragMode(bool checked)
{
    if (!checked) return;
    SetEditMode(LM::Mode_None);
    LM::ActionManager::Instance()->Record(LM::Mode_None);
    laneConfig->hide();
}

void MainWidget::OnMouseAction(LM::MouseAction evt)
{
#ifndef _DEBUG
    try
    {
#endif
        if (drawingSession != nullptr)
        {
            if (!drawingSession->Update(evt))
            {
                confirmEdit();
            }
        }
#ifndef _DEBUG
    }
    catch (CGAL::Failure_exception e)
    {
        spdlog::warn(e.what());
    }
    catch (std::exception e)
    {
        elegantlyHandleException(e);
    }
#endif
}

void MainWidget::OnKeyPress(LM::KeyPressAction evt)
{
    if (drawingSession == nullptr)
    {
        return;
    }
#ifndef _DEBUG
    try
    {
#endif
        switch (evt.key)
        {
        case Qt::Key_Escape:
            if (!drawingSession->Cancel())
            {
                quitEdit();
            }
            break;
        case Qt::Key_Space:
            {
                confirmEdit();
            }
            break;
        default:
            drawingSession->Update(evt);
            break;
        }

#ifndef _DEBUG
    }
    catch (CGAL::Failure_exception e)
    {
        spdlog::warn(e.what());
    }
    catch (std::exception e)
    {
        elegantlyHandleException(e);
    }
#endif
}

void MainWidget::confirmEdit()
{
    LM::ChangeTracker::Instance()->StartRecordEdit();
    bool cleanState = drawingSession->Complete();
    LM::ChangeTracker::Instance()->FinishRecordEdit(!cleanState);
    quitEdit();
}

void MainWidget::quitEdit()
{
    SetEditMode(editMode);
}

void MainWidget::toggleAntialiasing(bool enabled)
{
    auto fmt = mapViewGL->format();
    fmt.setSamples(enabled ? 4 : 1);
    mapViewGL->setFormat(fmt);
}

void MainWidget::Painted()
{
    auto t = QDateTime::currentMSecsSinceEpoch();
    nRepaints++;
    auto deltaMS = t - lastUpdateFPSMS;
    if (deltaMS > 1000)
    {
        auto fps = static_cast<double>(nRepaints) * 1000 / deltaMS;
        auto fpsInt = static_cast<int>(fps);
        auto displayStr = QString::fromStdString(std::to_string(fpsInt)) + tr(" FPS");
        emit FPSChanged(displayStr);

        lastUpdateFPSMS = t;
        nRepaints = 0;
    }
}

void MainWidget::Reset()
{
    laneConfig->Reset();
    pointerModeGroup->setExclusive(false);
    for (auto btn : pointerModeGroup->buttons())
    {
        btn->setChecked(false);
    }
    pointerModeGroup->setExclusive(true);
    // goto drag mode, but don't record
    SetEditMode(LM::Mode_None);
    laneConfig->hide();
}

void MainWidget::SetModeFromReplay(int mode)
{
    switch (mode)
    {
    case LM::Mode_Create:
        createModeButton->setChecked(true);
        break;
    case LM::Mode_CreateLanes:
        createLaneModeButton->setChecked(true);
        break;
    case LM::Mode_Modify:
        modifyModeButton->setChecked(true);
        break;
    case LM::Mode_Destroy:
        destroyModeButton->setChecked(true);
        break;
    default:
        dragModeButton->setChecked(true);
        break;
    }
}

void MainWidget::GoToSimulationMode(bool enabled)
{
    if (enabled)
    {
        dragModeButton->setChecked(true);
        //gotoDragMode();
    }
    for (auto btn : pointerModeGroup->buttons())
    {
        btn->setEnabled(!enabled);
    }
}

void MainWidget::SetEditMode(LM::EditMode aMode)
{
    editMode = aMode;

    if (drawingSession != nullptr)
    {
        delete drawingSession;
        drawingSession = nullptr;
    }
    switch (aMode)
    {
    case LM::Mode_Create:
        drawingSession = new RoadCreationSession();
        break;
    case LM::Mode_CreateLanes:
        drawingSession = new LanesCreationSession();
        break;
    case LM::Mode_Destroy:
        drawingSession = new RoadDestroySession();
        break;
    case LM::Mode_Modify:
        drawingSession = new RoadModificationSession();
        break;
    default:
        break;
    }
}

LM::EditMode MainWidget::GetEditMode() const
{
    return editMode;
}

void MainWidget::elegantlyHandleException(std::exception e)
{
    LM::ActionManager::Instance()->MarkException();
    auto msg = std::string(e.what()) + "\nReplayable at " + LM::ActionManager::Instance()->AutosavePath();
    auto quit = QMessageBox::question(this, "Quit now?",
        QString::fromStdString(msg), QMessageBox::Yes | QMessageBox::No);
    if (quit == QMessageBox::Yes)
    {
        QCoreApplication::quit();
    }
}

