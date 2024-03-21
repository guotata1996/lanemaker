// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "view.h"

#include <QtWidgets>
#include <QtMath>

#include "road_drawing.h"
#include "spdlog/spdlog.h"

GraphicsView::GraphicsView(View* v) : 
    QGraphicsView(), view(v)
{ 
    setSceneRect(-ViewPadding, -ViewPadding, ViewPadding, ViewPadding);
}

#if QT_CONFIG(wheelevent)
void GraphicsView::wheelEvent(QWheelEvent* e)
{
    if (e->modifiers() & Qt::ControlModifier) {
        if (e->angleDelta().y() > 0)
            view->zoomInBy(6);
        else
            view->zoomOutBy(6);
        e->accept();
    }
    else {
        QGraphicsView::wheelEvent(e);
    }
}
#endif

void GraphicsView::mousePressEvent(QMouseEvent* evt)
{
    QGraphicsView::mousePressEvent(evt);
    if (dragMode() == QGraphicsView::ScrollHandDrag)
    {
        return;
    }
    QPointF sceneEvt = mapToScene(evt->pos().x(), evt->pos().y());
    if (drawingSession == nullptr)
    {
        drawingSession = new RoadDrawingSession(this, sceneEvt);
        emit enableRoadEditingTool(true);
    }
    drawingSession->Update(evt);
}

void GraphicsView::mouseMoveEvent(QMouseEvent* evt)
{
    QGraphicsView::mouseMoveEvent(evt);

    if (drawingSession != nullptr)
    {
        drawingSession->Update(evt);
    }
}

void GraphicsView::keyPressEvent(QKeyEvent* evt)
{
    // End session, create road
    QGraphicsView::keyPressEvent(evt);
    if (drawingSession != nullptr && evt->key() == Qt::Key_Escape)
    {
        quitEdit();
    }
}

void GraphicsView::AdjustSceneRect()
{
    auto original = scene()->itemsBoundingRect();
    QRectF paded(original.left() - ViewPadding, original.top() - ViewPadding,
        original.width() + 2 * ViewPadding, original.height() + 2 * ViewPadding);
    setSceneRect(paded);
}

void GraphicsView::confirmEdit()
{
    drawingSession->Complete();
    AdjustSceneRect();

    quitEdit();
}

void GraphicsView::quitEdit()
{
    delete drawingSession;
    drawingSession = nullptr;

    emit enableRoadEditingTool(false);
}

View::View(const QString& name, QWidget* parent)
    : QFrame(parent)
{
    setFrameStyle(Sunken | StyledPanel);
    graphicsView = new GraphicsView(this);
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
    QHBoxLayout* roadEditingLayout = new QHBoxLayout;
    auto confirmButton = new QToolButton;
    confirmButton->setText("Confirm");
    roadEditingLayout->addWidget(confirmButton);
    auto cancelButton = new QToolButton;
    cancelButton->setText("Cancel");
    roadEditingLayout->addWidget(cancelButton);
    roadEditingTool = new QWidget(this);
    roadEditingTool->setLayout(roadEditingLayout);
    roadEditingTool->hide();

    QHBoxLayout* labelLayout = new QHBoxLayout;
    label2 = new QLabel(tr("Pointer Mode"));
    selectModeButton = new QToolButton;
    selectModeButton->setText(tr("Select"));
    selectModeButton->setCheckable(true);
    selectModeButton->setChecked(true);
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
    pointerModeGroup->addButton(selectModeButton);
    pointerModeGroup->addButton(dragModeButton);

    labelLayout->addWidget(roadEditingTool);
    labelLayout->addStretch();
    labelLayout->addWidget(label2);
    labelLayout->addWidget(selectModeButton);
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

    connect(resetButton, &QAbstractButton::clicked, this, &View::resetView);
    connect(zoomSlider, &QAbstractSlider::valueChanged, this, &View::setupMatrix);
    connect(rotateSlider, &QAbstractSlider::valueChanged, this, &View::setupMatrix);
    connect(graphicsView->verticalScrollBar(), &QAbstractSlider::valueChanged,
        this, &View::setResetButtonEnabled);
    connect(graphicsView->horizontalScrollBar(), &QAbstractSlider::valueChanged,
        this, &View::setResetButtonEnabled);
    connect(selectModeButton, &QAbstractButton::toggled, this, &View::togglePointerMode);
    connect(dragModeButton, &QAbstractButton::toggled, this, &View::togglePointerMode);
    connect(antialiasButton, &QAbstractButton::toggled, this, &View::toggleAntialiasing);
    connect(rotateLeftIcon, &QAbstractButton::clicked, this, &View::rotateLeft);
    connect(rotateRightIcon, &QAbstractButton::clicked, this, &View::rotateRight);
    connect(zoomInIcon, &QAbstractButton::clicked, this, &View::zoomIn);
    connect(zoomOutIcon, &QAbstractButton::clicked, this, &View::zoomOut);

    connect(graphicsView, &GraphicsView::enableRoadEditingTool, this, &View::showRoadEditingTool);
    connect(confirmButton, &QAbstractButton::clicked, graphicsView, &GraphicsView::confirmEdit);
    connect(cancelButton, &QAbstractButton::clicked, graphicsView, &GraphicsView::quitEdit);

    setupMatrix();
}

QGraphicsView* View::view() const
{
    return static_cast<QGraphicsView*>(graphicsView);
}

void View::resetView()
{
    zoomSlider->setValue(250);
    rotateSlider->setValue(0);
    setupMatrix();
    graphicsView->ensureVisible(QRectF(0, 0, 0, 0));

    resetButton->setEnabled(false);
}

void View::setResetButtonEnabled()
{
    resetButton->setEnabled(true);
}

void View::setupMatrix()
{
    qreal scale = qPow(qreal(2), (zoomSlider->value() - 250) / qreal(50));

    QTransform matrix;
    matrix.scale(scale, scale);
    matrix.rotate(rotateSlider->value());

    graphicsView->setTransform(matrix);
    setResetButtonEnabled();
}

void View::togglePointerMode()
{
    graphicsView->setDragMode(selectModeButton->isChecked()
        ? QGraphicsView::RubberBandDrag
        : QGraphicsView::ScrollHandDrag);
    graphicsView->setInteractive(selectModeButton->isChecked());
}

void View::toggleAntialiasing()
{
    graphicsView->setRenderHint(QPainter::Antialiasing, antialiasButton->isChecked());
}

void View::zoomIn()
{
    zoomSlider->setValue(zoomSlider->value() + 1);
}

void View::zoomOut()
{
    zoomSlider->setValue(zoomSlider->value() - 1);
}

void View::zoomInBy(int level)
{
    zoomSlider->setValue(zoomSlider->value() + level);
}

void View::zoomOutBy(int level)
{
    zoomSlider->setValue(zoomSlider->value() - level);
}

void View::rotateLeft()
{
    rotateSlider->setValue(rotateSlider->value() - 10);
}

void View::rotateRight()
{
    rotateSlider->setValue(rotateSlider->value() + 10);
}

void View::showRoadEditingTool(bool show)
{
    roadEditingTool->setHidden(!show);
}