// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "main_widget.h"
#include "map_view.h"
#include "map_view_gl.h"
#include "mainwindow.h"
#include "action_manager.h"
#include "road_drawing.h"
#include "change_tracker.h"

#include "CreateRoadOptionWidget.h"
#include "spdlog/spdlog.h"

#include <QtWidgets>
#include <QtMath>

extern SectionProfileConfigWidget* g_createRoadOption;
int8_t g_createRoadElevationOption;

MainWidget::MainWidget(QGraphicsScene* scene, QWidget* parent)
    : QFrame(parent), createRoadOption(new SectionProfileConfigWidget),
    displayScaleTimer(new QTimer(this))
{
    setFrameStyle(Sunken | StyledPanel);
    mapView = new MapView(this, scene);
    mapView->setRenderHint(QPainter::Antialiasing, false);
    mapView->setDragMode(QGraphicsView::RubberBandDrag);
    mapView->setOptimizationFlags(QGraphicsView::DontSavePainterState);
    mapView->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    mapView->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    mapView->setMouseTracking(true);

    QSurfaceFormat format;
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setVersion(3, 3);
    format.setSamples(4);	// enable multisampling (antialiasing)
    format.setDepthBufferSize(8);

    mapViewGL = new RoadRunner::MapViewGL;
    mapViewGL->setFormat(format);

    QWidget* container = QWidget::createWindowContainer(mapViewGL, this);
    container->setFocusPolicy(Qt::TabFocus);
    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

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

    labelLayout->addWidget(createRoadOption);
    QSizePolicy sp_retain = createRoadOption->sizePolicy();
    sp_retain.setRetainSizeWhenHidden(true);
    createRoadOption->setSizePolicy(sp_retain);
    createRoadOption->hide();
    g_createRoadOption = createRoadOption;
    labelLayout->addSpacing(100);
    labelLayout->addWidget(createModeButton);
    labelLayout->addWidget(createLaneModeButton);
    labelLayout->addWidget(modifyModeButton);
    labelLayout->addWidget(destroyModeButton);
    labelLayout->addWidget(dragModeButton);
    labelLayout->addStretch();

    QVBoxLayout* topLayout = new QVBoxLayout;
    topLayout->addLayout(labelLayout);
    topLayout->addWidget(container);
    setLayout(topLayout);

    displayScaleTimer->setSingleShot(true);

    connect(createModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoCreateRoadMode);
    connect(createLaneModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoCreateLaneMode);
    connect(destroyModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoDestroyMode);
    connect(modifyModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoModifyMode);
    connect(dragModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoDragMode);
    connect(displayScaleTimer, &QTimer::timeout, mapView, &MapView::hideScale);
    connect(mapViewGL, &RoadRunner::MapViewGL::MousePerformedAction, this, &MainWidget::OnMouseMove);
    connect(mapViewGL, &RoadRunner::MapViewGL::KeyPerformedAction, this, &MainWidget::OnKeyPressed);
    Reset();
}

QGraphicsView* MainWidget::view() const
{
    return static_cast<QGraphicsView*>(mapView);
}


void MainWidget::gotoCreateRoadMode(bool checked)
{
    if (!checked) return;
    SetEditMode(RoadRunner::Mode_Create);
    RoadRunner::ActionManager::Instance()->Record(RoadRunner::Mode_Create);
    createRoadOption->GotoRoadMode();
    emit InReadOnlyMode(false);
}

void MainWidget::gotoCreateLaneMode(bool checked)
{
    if (!checked) return;
    SetEditMode(RoadRunner::Mode_CreateLanes);
    RoadRunner::ActionManager::Instance()->Record(RoadRunner::Mode_CreateLanes);
    createRoadOption->GotoLaneMode();
    emit InReadOnlyMode(false);
}

void MainWidget::gotoDestroyMode(bool checked)
{
    if (!checked) return;
    SetEditMode(RoadRunner::Mode_Destroy);
    RoadRunner::ActionManager::Instance()->Record(RoadRunner::Mode_Destroy);
    createRoadOption->hide();
    emit InReadOnlyMode(false);
}

void MainWidget::gotoModifyMode(bool checked)
{
    if (!checked) return;
    SetEditMode(RoadRunner::Mode_Modify);
    RoadRunner::ActionManager::Instance()->Record(RoadRunner::Mode_Modify);
    createRoadOption->GotoRoadMode();
    emit InReadOnlyMode(false);
}

void MainWidget::gotoDragMode(bool checked)
{
    if (!checked) return;
    SetEditMode(RoadRunner::Mode_None);
    RoadRunner::ActionManager::Instance()->Record(RoadRunner::Mode_None);
    createRoadOption->hide();
    emit InReadOnlyMode(true);
}

void MainWidget::OnMouseMove(RoadRunner::MouseAction evt)
{
    if (drawingSession != nullptr)
    {
        if (!drawingSession->Update(evt))
        {
            confirmEdit();
        }
    }
}

void MainWidget::OnKeyPressed(RoadRunner::KeyPressAction evt)
{
    switch (evt.key)
    {
    case Qt::Key_Escape:
        quitEdit();
        break;
    case Qt::Key_Return:
        if (drawingSession != nullptr)
        {
            confirmEdit();
        }
        break;
    case Qt::Key_I:
        // TODO
        break;
    }
}

void MainWidget::confirmEdit()
{
    RoadRunner::ChangeTracker::Instance()->StartRecordEdit();
    bool cleanState = drawingSession->Complete();
    RoadRunner::ChangeTracker::Instance()->FinishRecordEdit(!cleanState);
    quitEdit();
}

void MainWidget::quitEdit()
{
    SetEditMode(editMode);
}

void MainWidget::toggleAntialiasing(bool enabled)
{
    mapView->setRenderHint(QPainter::Antialiasing, enabled);
}

void MainWidget::PostEditActions()
{
    mapView->PostEditActions();
}

void MainWidget::SetHovering(QString a)
{
    emit HoveringChanged(a);
}

void MainWidget::SetBackgroundImage(const QPixmap& image)
{
    mapView->SetBackground(image);
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
    createRoadOption->Reset();
    pointerModeGroup->setExclusive(false);
    for (auto btn : pointerModeGroup->buttons())
    {
        btn->setChecked(false);
    }
    pointerModeGroup->setExclusive(true);
    gotoDragMode();
    mapView->ResetSceneRect();
    displayScaleTimer->stop();
    mapView->showScale(); // Show scale upon start / newMap
}

void MainWidget::RecordViewTransform()
{
    // TODO
}

void MainWidget::SetViewFromReplay(double zoomSliderVal, double rotateSliderVal)
{
    // TODO
}

void MainWidget::SetModeFromReplay(int mode)
{
    switch (mode)
    {
    case RoadRunner::Mode_Create:
        createModeButton->setChecked(true);
        break;
    case RoadRunner::Mode_CreateLanes:
        createLaneModeButton->setChecked(true);
        break;
    case RoadRunner::Mode_Modify:
        modifyModeButton->setChecked(true);
        break;
    case RoadRunner::Mode_Destroy:
        destroyModeButton->setChecked(true);
        break;
    default:
        dragModeButton->setChecked(true);
        break;
    }
}

void MainWidget::SetElevationFromReplay(int8_t elevationSetting)
{
    // TODO
}

void MainWidget::SetEditMode(RoadRunner::EditMode aMode)
{
    editMode = aMode;

    if (drawingSession != nullptr)
    {
        delete drawingSession;
        drawingSession = nullptr;
    }
    switch (aMode)
    {
    case RoadRunner::Mode_Create:
        drawingSession = new RoadCreationSession();
        break;
    case RoadRunner::Mode_CreateLanes:
        drawingSession = new LanesCreationSession();
        break;
    case RoadRunner::Mode_Destroy:
        drawingSession = new RoadDestroySession();
        break;
    case RoadRunner::Mode_Modify:
        drawingSession = new RoadModificationSession();
        break;
    default:
        break;
    }
}

