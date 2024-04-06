// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "view.h"
#include "road_drawing.h"
#include "CreateRoadOptionWidget.h"
#include "spdlog/spdlog.h"

#include <sstream>
#include <QtWidgets>
#include <QtMath>

std::weak_ptr<RoadRunner::Road> g_PointerRoad;
double g_PointerRoadS;

GraphicsView::GraphicsView(View* v) : 
    QGraphicsView(), view(v)
{ 
    setSceneRect(-ViewPadding, -ViewPadding, 2 * ViewPadding, 2 * ViewPadding);
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

void GraphicsView::SetEditMode(EditMode aMode)
{
    editMode = aMode;

    if (drawingSession != nullptr)
    {
        delete drawingSession;
        drawingSession = nullptr;
    }
    switch (aMode)
    {
        case Mode_Create:
            drawingSession = new RoadCreationSession(this);
            break;
        case Mode_Destroy:
            drawingSession = new RoadDestroySession(this);
            break;
        default:
            break;
    }
}

void GraphicsView::mousePressEvent(QMouseEvent* evt)
{
    QGraphicsView::mousePressEvent(evt);
    if (editMode != Mode_None)
    {
        if (!drawingSession->Update(evt))
        {
            confirmEdit();
        }
    }
}

void GraphicsView::mouseMoveEvent(QMouseEvent* evt)
{
    QGraphicsView::mouseMoveEvent(evt);

    if (editMode != Mode_None)
    {
        drawingSession->Update(evt);
    }
}

void GraphicsView::keyPressEvent(QKeyEvent* evt)
{
    QGraphicsView::keyPressEvent(evt);
    if (evt->key() == Qt::Key_Escape)
    {
        quitEdit();
    }
    else if (evt->key() == Qt::Key_Return)
    {
        confirmEdit();
    }
    else if (evt->key() == Qt::Key_I)
    {
        auto g_road = g_PointerRoad.lock();
        if (g_road != nullptr)
        {
            spdlog::info("Road {0}: Length= {1:.3f}", g_road->ID(), g_road->Length());
            g_road->profile.PrintDetails();
        }
    }
}

void GraphicsView::drawForeground(QPainter* painter, const QRectF& rect)
{
    QGraphicsView::drawForeground(painter, rect);

    auto g_road = g_PointerRoad.lock();
    if (g_road != nullptr)
    {
        painter->save();

        QFont font = painter->font();
        font.setPointSize(20);
        painter->setFont(font);

        painter->setWorldMatrixEnabled(false);
        QRectF vpRect = viewport()->rect();

        std::stringstream ss;
        ss << "Road " << g_road->ID() << " @ " << g_PointerRoadS;
        QString txt = QString::fromStdString(ss.str());
        painter->drawText(vpRect.bottomLeft(), txt);
        painter->setWorldMatrixEnabled(true);
        painter->restore();
    }

    viewport()->update();
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
    quitEdit();

    AdjustSceneRect();
}

void GraphicsView::quitEdit()
{
    SetEditMode(editMode);
}

View::View(const QString& name, QWidget* parent)
    : QFrame(parent), createRoadOption(new CreateRoadOptionWidget)
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
    QHBoxLayout* labelLayout = new QHBoxLayout;
    label2 = new QLabel(tr("Pointer Mode"));
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
    createRoadOption->hide();
    labelLayout->addStretch();
    labelLayout->addWidget(label2);
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

    connect(resetButton, &QAbstractButton::clicked, this, &View::resetView);
    connect(zoomSlider, &QAbstractSlider::valueChanged, this, &View::setupMatrix);
    connect(rotateSlider, &QAbstractSlider::valueChanged, this, &View::setupMatrix);
    connect(graphicsView->verticalScrollBar(), &QAbstractSlider::valueChanged,
        this, &View::setResetButtonEnabled);
    connect(graphicsView->horizontalScrollBar(), &QAbstractSlider::valueChanged,
        this, &View::setResetButtonEnabled);
    connect(createModeButton, &QAbstractButton::toggled, this, &View::gotoCreateMode);
    connect(destroyModeButton, &QAbstractButton::toggled, this, &View::gotoDestroyMode);
    connect(dragModeButton, &QAbstractButton::toggled, this, &View::gotoDragMode);
    connect(antialiasButton, &QAbstractButton::toggled, this, &View::toggleAntialiasing);
    connect(rotateLeftIcon, &QAbstractButton::clicked, this, &View::rotateLeft);
    connect(rotateRightIcon, &QAbstractButton::clicked, this, &View::rotateRight);
    connect(zoomInIcon, &QAbstractButton::clicked, this, &View::zoomIn);
    connect(zoomOutIcon, &QAbstractButton::clicked, this, &View::zoomOut);

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
    matrix.scale(scale, -scale);
    matrix.rotate(rotateSlider->value());

    graphicsView->setTransform(matrix);
    setResetButtonEnabled();
}

void View::gotoCreateMode()
{
    graphicsView->setDragMode(QGraphicsView::RubberBandDrag);
    graphicsView->setInteractive(true);
    graphicsView->SetEditMode(GraphicsView::Mode_Create);
    createRoadOption->show();
}

void View::gotoDestroyMode()
{
    graphicsView->setDragMode(QGraphicsView::RubberBandDrag);
    graphicsView->setInteractive(true);
    graphicsView->SetEditMode(GraphicsView::Mode_Destroy);
    createRoadOption->hide();
}

void View::gotoDragMode()
{
    graphicsView->setDragMode(QGraphicsView::ScrollHandDrag);
    graphicsView->setInteractive(false);
    graphicsView->SetEditMode(GraphicsView::Mode_None);
    createRoadOption->hide();
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