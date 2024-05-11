// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "main_widget.h"
#include "map_view.h"
#include "mainwindow.h"

#include "CreateRoadOptionWidget.h"
#include "spdlog/spdlog.h"

#include <QtWidgets>
#include <QtMath>

double g_zoom;

MainWidget::MainWidget(const QString& name, QWidget* parent)
    : QFrame(parent), createRoadOption(new CreateRoadOptionWidget)
{
    setFrameStyle(Sunken | StyledPanel);
    graphicsView = new MapView(this);
    graphicsView->setRenderHint(QPainter::Antialiasing, false);
    graphicsView->setDragMode(QGraphicsView::RubberBandDrag);
    graphicsView->setOptimizationFlags(QGraphicsView::DontSavePainterState);
    graphicsView->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    graphicsView->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    graphicsView->setMouseTracking(true);

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
    createModeButton->setText(tr("Create"));
    createModeButton->setCheckable(true);
    createModeButton->setChecked(false);
    destroyModeButton = new QToolButton;
    destroyModeButton->setText(tr("Destroy"));
    destroyModeButton->setCheckable(true);
    destroyModeButton->setChecked(false);
    dragModeButton = new QToolButton;
    dragModeButton->setText(tr("Drag"));
    dragModeButton->setCheckable(true);
    dragModeButton->setChecked(false);
    antialiasButton = new QToolButton;
    antialiasButton->setText(tr("Antialiasing"));
    antialiasButton->setCheckable(true);
    antialiasButton->setChecked(false);

    QButtonGroup* pointerModeGroup = new QButtonGroup(this);
    pointerModeGroup->setExclusive(true);
    pointerModeGroup->addButton(createModeButton);
    pointerModeGroup->addButton(destroyModeButton);
    pointerModeGroup->addButton(dragModeButton);

    labelLayout->addWidget(createRoadOption);
    QSizePolicy sp_retain = createRoadOption->sizePolicy();
    sp_retain.setRetainSizeWhenHidden(true);
    createRoadOption->setSizePolicy(sp_retain);
    createRoadOption->hide();
    labelLayout->addSpacing(50);
    labelLayout->addWidget(new QLabel(tr("Edit Mode")));
    labelLayout->addWidget(createModeButton);
    labelLayout->addWidget(destroyModeButton);
    labelLayout->addWidget(dragModeButton);
    labelLayout->addStretch();
    labelLayout->addWidget(antialiasButton);

    QGridLayout* topLayout = new QGridLayout;
    topLayout->addLayout(labelLayout, 0, 0);
    topLayout->addWidget(graphicsView, 1, 0);
    topLayout->addLayout(zoomSliderLayout, 1, 1);
    topLayout->addLayout(rotateSliderLayout, 2, 0);
    topLayout->addWidget(resetButton, 2, 1);
    setLayout(topLayout);

    connect(resetButton, &QAbstractButton::clicked, this, &MainWidget::resetView);
    connect(zoomSlider, &QAbstractSlider::valueChanged, this, &MainWidget::setupMatrix);
    connect(rotateSlider, &QAbstractSlider::valueChanged, this, &MainWidget::setupMatrix);
    connect(graphicsView->verticalScrollBar(), &QAbstractSlider::valueChanged,
        this, &MainWidget::setResetButtonEnabled);
    connect(graphicsView->horizontalScrollBar(), &QAbstractSlider::valueChanged,
        this, &MainWidget::setResetButtonEnabled);
    connect(createModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoCreateMode);
    connect(destroyModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoDestroyMode);
    connect(dragModeButton, &QAbstractButton::toggled, this, &MainWidget::gotoDragMode);
    connect(antialiasButton, &QAbstractButton::toggled, this, &MainWidget::toggleAntialiasing);
    connect(rotateLeftIcon, &QAbstractButton::clicked, this, &MainWidget::rotateLeft);
    connect(rotateRightIcon, &QAbstractButton::clicked, this, &MainWidget::rotateRight);
    connect(zoomInIcon, &QAbstractButton::clicked, this, &MainWidget::zoomIn);
    connect(zoomOutIcon, &QAbstractButton::clicked, this, &MainWidget::zoomOut);

    setupMatrix();
}

QGraphicsView* MainWidget::view() const
{
    return static_cast<QGraphicsView*>(graphicsView);
}

void MainWidget::resetView()
{
    zoomSlider->setValue(250);
    rotateSlider->setValue(0);
    setupMatrix();
    graphicsView->ensureVisible(QRectF(0, 0, 0, 0));

    resetButton->setEnabled(false);
}

void MainWidget::setResetButtonEnabled()
{
    resetButton->setEnabled(true);
}

void MainWidget::setupMatrix()
{
    qreal scale = qPow(qreal(2), (zoomSlider->value() - 250) / qreal(50));
    g_zoom = scale;

    QTransform matrix;
    matrix.scale(scale, -scale);
    matrix.rotate(rotateSlider->value());

    graphicsView->setTransform(matrix);
    setResetButtonEnabled();
}

void MainWidget::gotoCreateMode()
{
    graphicsView->setDragMode(QGraphicsView::RubberBandDrag);
    graphicsView->setInteractive(true);
    graphicsView->SetEditMode(MapView::Mode_Create);
    createRoadOption->show();
}

void MainWidget::gotoDestroyMode()
{
    graphicsView->setDragMode(QGraphicsView::RubberBandDrag);
    graphicsView->setInteractive(true);
    graphicsView->SetEditMode(MapView::Mode_Destroy);
    createRoadOption->hide();
}

void MainWidget::gotoDragMode()
{
    graphicsView->setDragMode(QGraphicsView::ScrollHandDrag);
    graphicsView->setInteractive(false);
    graphicsView->SetEditMode(MapView::Mode_None);
    createRoadOption->hide();
}

void MainWidget::toggleAntialiasing()
{
    graphicsView->setRenderHint(QPainter::Antialiasing, antialiasButton->isChecked());
}

void MainWidget::zoomIn()
{
    zoomSlider->setValue(zoomSlider->value() + 1);
}

void MainWidget::zoomOut()
{
    zoomSlider->setValue(zoomSlider->value() - 1);
}

void MainWidget::zoomInBy(int level)
{
    zoomSlider->setValue(zoomSlider->value() + level);
}

void MainWidget::zoomOutBy(int level)
{
    zoomSlider->setValue(zoomSlider->value() - level);
}

void MainWidget::rotateLeft()
{
    rotateSlider->setValue(rotateSlider->value() - 10);
}

void MainWidget::rotateRight()
{
    rotateSlider->setValue(rotateSlider->value() + 10);
}

void MainWidget::AdjustSceneRect()
{
    graphicsView->AdjustSceneRect();
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