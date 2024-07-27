// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "main_widget.h"
#include "map_view.h"
#include "mainwindow.h"
#include "action_manager.h"

#include "CreateRoadOptionWidget.h"
#include "spdlog/spdlog.h"

#include <QtWidgets>
#include <QtMath>

extern SectionProfileConfigWidget* g_createRoadOption;

MainWidget::MainWidget(const QString& name, QWidget* parent)
    : QFrame(parent), createRoadOption(new SectionProfileConfigWidget),
    displayScaleTimer(new QTimer(this))
{
    setFrameStyle(Sunken | StyledPanel);
    mapView = new MapView(this);
    mapView->setRenderHint(QPainter::Antialiasing, false);
    mapView->setDragMode(QGraphicsView::RubberBandDrag);
    mapView->setOptimizationFlags(QGraphicsView::DontSavePainterState);
    mapView->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    mapView->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    mapView->setMouseTracking(true);

    int size = style()->pixelMetric(QStyle::PM_ToolBarIconSize);
    QSize iconSize(size, size);

    QToolButton* zoomInIcon = new QToolButton;
    zoomInIcon->setAutoRepeat(true);
    zoomInIcon->setAutoRepeatInterval(33);
    zoomInIcon->setAutoRepeatDelay(0);
    zoomInIcon->setIcon(QPixmap(":/icons/zoomin.png"));
    zoomInIcon->setIconSize(iconSize);
    QToolButton* zoomOutIcon = new QToolButton;
    zoomOutIcon->setAutoRepeat(true);
    zoomOutIcon->setAutoRepeatInterval(33);
    zoomOutIcon->setAutoRepeatDelay(0);
    zoomOutIcon->setIcon(QPixmap(":/icons/zoomout.png"));
    zoomOutIcon->setIconSize(iconSize);
    zoomSlider = new QSlider;
    zoomSlider->setMinimum(0);
    zoomSlider->setMaximum(500);
    zoomSlider->setValue(250);
    zoomSlider->setTickPosition(QSlider::TicksRight);

    // Zoom slider layout
    QVBoxLayout* zoomSliderLayout = new QVBoxLayout;
    zoomSliderLayout->addWidget(zoomInIcon);
    zoomSliderLayout->addWidget(zoomSlider);
    zoomSliderLayout->addWidget(zoomOutIcon);

    QToolButton* rotateLeftIcon = new QToolButton;
    rotateLeftIcon->setIcon(QPixmap(":/icons/rotateleft.png"));
    rotateLeftIcon->setIconSize(iconSize);
    QToolButton* rotateRightIcon = new QToolButton;
    rotateRightIcon->setIcon(QPixmap(":/icons/rotateright.png"));
    rotateRightIcon->setIconSize(iconSize);
    rotateSlider = new QSlider;
    rotateSlider->setOrientation(Qt::Horizontal);
    rotateSlider->setMinimum(-360);
    rotateSlider->setMaximum(360);
    rotateSlider->setValue(0);
    rotateSlider->setTickPosition(QSlider::TicksBelow);

    // Rotate slider layout
    QHBoxLayout* rotateSliderLayout = new QHBoxLayout;
    rotateSliderLayout->addWidget(rotateLeftIcon);
    rotateSliderLayout->addWidget(rotateSlider);
    rotateSliderLayout->addWidget(rotateRightIcon);

    resetButton = new QToolButton;
    resetButton->setText(tr("0"));
    resetButton->setEnabled(false);

    // Label layout
    QHBoxLayout* labelLayout = new QHBoxLayout;
    createModeButton = new QToolButton;
    createModeButton->setToolTip(tr("Road Mode"));
    createModeButton->setIcon(QPixmap(":/icons/road_mode.png"));
    createModeButton->setIconSize(iconSize);
    createModeButton->setCheckable(true);
    createModeButton->setChecked(false);
    createLaneModeButton = new QToolButton;
    createLaneModeButton->setToolTip(tr("Lane Mode"));
    createLaneModeButton->setIcon(QPixmap(":/icons/lane_mode.png"));
    createLaneModeButton->setIconSize(iconSize);
    createLaneModeButton->setCheckable(true);
    createLaneModeButton->setChecked(false);
    destroyModeButton = new QToolButton;
    destroyModeButton->setToolTip(tr("Destroy Mode"));
    destroyModeButton->setIcon(QPixmap(":/icons/destroy_mode.png"));
    destroyModeButton->setIconSize(iconSize);
    destroyModeButton->setCheckable(true);
    destroyModeButton->setChecked(false);
    modifyModeButton = new QToolButton;
    modifyModeButton->setToolTip(tr("Modify Mode"));
    modifyModeButton->setIcon(QPixmap(":/icons/modify_mode.PNG"));
    modifyModeButton->setIconSize(iconSize);
    modifyModeButton->setCheckable(true);
    modifyModeButton->setChecked(false);
    dragModeButton = new QToolButton;
    dragModeButton->setToolTip(tr("Drag Mode"));
    dragModeButton->setIcon(QPixmap(":/icons/view_mode.png"));
    dragModeButton->setIconSize(iconSize);
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
    labelLayout->addSpacing(50);
    labelLayout->addWidget(new QLabel(tr("Edit Mode")));
    labelLayout->addWidget(createModeButton);
    labelLayout->addWidget(createLaneModeButton);
    labelLayout->addWidget(modifyModeButton);
    labelLayout->addWidget(destroyModeButton);
    labelLayout->addWidget(dragModeButton);
    labelLayout->addStretch();

    QGridLayout* topLayout = new QGridLayout;
    topLayout->addLayout(labelLayout, 0, 0);
    topLayout->addWidget(mapView, 1, 0);
    topLayout->addLayout(zoomSliderLayout, 1, 1);
    topLayout->addLayout(rotateSliderLayout, 2, 0);
    topLayout->addWidget(resetButton, 2, 1);
    setLayout(topLayout);

    displayScaleTimer->setSingleShot(true);

    connect(resetButton, &QAbstractButton::clicked, this, &MainWidget::resetView);
    connect(zoomSlider, &QAbstractSlider::valueChanged, this, &MainWidget::setupMatrix);
    connect(rotateSlider, &QAbstractSlider::valueChanged, this, &MainWidget::setupMatrix);
    connect(zoomSlider, &QAbstractSlider::valueChanged, this, &MainWidget::RecordViewTransform);
    connect(rotateSlider, &QAbstractSlider::valueChanged, this, &MainWidget::RecordViewTransform);
    connect(mapView->verticalScrollBar(), &QAbstractSlider::valueChanged,
        this, &MainWidget::setResetButtonEnabled);
    connect(mapView->horizontalScrollBar(), &QAbstractSlider::valueChanged,
        this, &MainWidget::setResetButtonEnabled);
    connect(createModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoCreateRoadMode);
    connect(createLaneModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoCreateLaneMode);
    connect(destroyModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoDestroyMode);
    connect(modifyModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoModifyMode);
    connect(dragModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoDragMode);
    connect(rotateLeftIcon, &QAbstractButton::clicked, this, &MainWidget::rotateLeft);
    connect(rotateRightIcon, &QAbstractButton::clicked, this, &MainWidget::rotateRight);
    connect(zoomInIcon, &QAbstractButton::clicked, this, &MainWidget::zoomIn);
    connect(zoomOutIcon, &QAbstractButton::clicked, this, &MainWidget::zoomOut);
    connect(displayScaleTimer, &QTimer::timeout, mapView, &MapView::hideScale);

    Reset();
}

QGraphicsView* MainWidget::view() const
{
    return static_cast<QGraphicsView*>(mapView);
}

void MainWidget::resetView()
{
    zoomSlider->setValue(250);
    rotateSlider->setValue(0);
    setupMatrix();
    mapView->ensureVisible(QRectF(0, 0, 0, 0));

    resetButton->setEnabled(false);
}

void MainWidget::setResetButtonEnabled()
{
    resetButton->setEnabled(true);
}

void MainWidget::setupMatrix()
{
    qreal scale = qPow(qreal(2), (zoomSlider->value() - 250) / qreal(50));

    QTransform matrix;
    matrix.scale(scale, -scale);
    matrix.rotate(rotateSlider->value());

    mapView->setTransform(matrix);
    setResetButtonEnabled();

    mapView->showScale();
    displayScaleTimer->start(1000);
}

void MainWidget::gotoCreateRoadMode()
{
    mapView->setDragMode(QGraphicsView::NoDrag);
    mapView->setInteractive(true);
    mapView->SetEditMode(MapView::Mode_Create);
    RoadRunner::ActionManager::Instance()->Record(MapView::Mode_Create);
    createRoadOption->GotoRoadMode();
    emit InReadOnlyMode(false);
}

void MainWidget::gotoCreateLaneMode()
{
    mapView->setDragMode(QGraphicsView::NoDrag);
    mapView->setInteractive(true);
    mapView->SetEditMode(MapView::Mode_CreateLanes);
    RoadRunner::ActionManager::Instance()->Record(MapView::Mode_CreateLanes);
    createRoadOption->GotoLaneMode();
    emit InReadOnlyMode(false);
}

void MainWidget::gotoDestroyMode()
{
    mapView->setDragMode(QGraphicsView::NoDrag);
    mapView->setInteractive(true);
    mapView->SetEditMode(MapView::Mode_Destroy);
    RoadRunner::ActionManager::Instance()->Record(MapView::Mode_Destroy);
    createRoadOption->hide();
    emit InReadOnlyMode(false);
}

void MainWidget::gotoModifyMode()
{
    mapView->setDragMode(QGraphicsView::NoDrag);
    mapView->setInteractive(true);
    mapView->SetEditMode(MapView::Mode_Modify);
    RoadRunner::ActionManager::Instance()->Record(MapView::Mode_Modify);
    createRoadOption->GotoRoadMode();
    emit InReadOnlyMode(false);
}

void MainWidget::gotoDragMode()
{
    mapView->setDragMode(QGraphicsView::ScrollHandDrag);
    mapView->setInteractive(false);
    mapView->SetEditMode(MapView::Mode_None);
    RoadRunner::ActionManager::Instance()->Record(MapView::Mode_None);
    createRoadOption->hide();
    emit InReadOnlyMode(true);
}

void MainWidget::toggleAntialiasing(bool enabled)
{
    mapView->setRenderHint(QPainter::Antialiasing, enabled);
}

void MainWidget::zoomIn()
{
    zoomSlider->setValue(zoomSlider->value() + 1);
    RecordViewTransform();
}

void MainWidget::zoomOut()
{
    zoomSlider->setValue(zoomSlider->value() - 1);
    RecordViewTransform();
}

void MainWidget::zoomInBy(int level)
{
    zoomSlider->setValue(zoomSlider->value() + level);
    RecordViewTransform();
}

void MainWidget::zoomOutBy(int level)
{
    zoomSlider->setValue(zoomSlider->value() - level);
    RecordViewTransform();
}

void MainWidget::rotateLeft()
{
    rotateSlider->setValue(rotateSlider->value() - 10);
    RecordViewTransform();
}

void MainWidget::rotateRight()
{
    rotateSlider->setValue(rotateSlider->value() + 10);
    RecordViewTransform();
}

void MainWidget::AdjustSceneRect()
{
    mapView->AdjustSceneRect();
}

void MainWidget::SetHovering(QString a)
{
    emit HoveringChanged(a);
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
    resetView(); // Make sure to RecordViewTransform()
    mapView->ResetSceneRect();
    displayScaleTimer->stop();
    mapView->showScale(); // Show scale upon start / newMap
}

void MainWidget::RecordViewTransform()
{
    if (zoomSlider != nullptr && rotateSlider != nullptr)
    {
        RoadRunner::ActionManager::Instance()->Record(
            zoomSlider->value(), rotateSlider->value(),
            mapView->horizontalScrollBar()->value(), mapView->verticalScrollBar()->value());
    }
}

void MainWidget::SetViewFromReplay(double zoomSliderVal, double rotateSliderVal)
{
    zoomSlider->setValue(zoomSliderVal);
    rotateSlider->setValue(rotateSliderVal);
}

void MainWidget::SetModeFromReplay(int mode)
{
    switch (mode)
    {
    case MapView::EditMode::Mode_Create:
        createModeButton->setChecked(true);
        break;
    case MapView::EditMode::Mode_CreateLanes:
        createLaneModeButton->setChecked(true);
        break;
    case MapView::EditMode::Mode_Modify:
        modifyModeButton->setChecked(true);
        break;
    case MapView::EditMode::Mode_Destroy:
        destroyModeButton->setChecked(true);
        break;
    default:
        dragModeButton->setChecked(true);
        break;
    }
}