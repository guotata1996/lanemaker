#include "view.h"
#include "road_drawing.h"
#include "road_graphics.h"

#include <sstream>
#include <qevent.h>

std::weak_ptr<RoadRunner::Road> g_PointerRoad;
double g_PointerRoadS; /*Continous between 0 and Length() if g_PointerRoad is valid*/


MapView::MapView(MainWidget* v) :
    QGraphicsView(), view(v)
{
    setSceneRect(-ViewPadding, -ViewPadding, 2 * ViewPadding, 2 * ViewPadding);
}

#if QT_CONFIG(wheelevent)
void MapView::wheelEvent(QWheelEvent* e)
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

void MapView::SetEditMode(EditMode aMode)
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

void MapView::SnapCursor(const QPoint& viewPos)
{
    g_PointerRoad.reset();
    double r = CustomCursorItem::SnapRadiusPx;
    auto candidates = items(viewPos.x() - r, viewPos.y() - r, 2 * r, 2 * r);
    std::set<std::shared_ptr<RoadRunner::Road>> nearbyRoads;
    for (auto item : candidates)
    {
        while (item != nullptr)
        {
            auto laneGraphicsItem = dynamic_cast<RoadRunner::LaneSegmentGraphics*>(item);
            if (laneGraphicsItem != nullptr)
            {
                if (laneGraphicsItem->SnapCursor(mapToScene(viewPos)))
                {
                    return;
                }
                nearbyRoads.insert(laneGraphicsItem->Road());
                break;
            }
            item = item->parentItem();
        }
    }

    double closestS;
    std::shared_ptr<RoadRunner::Road> closestRoad;
    QVector2D scenePos(mapToScene(viewPos));
    double closestDistance = 1e9;
    QPointF targetScenePos;
    for (auto& nearbyRoad : nearbyRoads)
    {
        for (bool start : {false, true})
        {
            double x, y;
            nearbyRoad->GetEndPoint(start, x, y);
            QVector2D endPoint(x, y);
            double dist = scenePos.distanceToPoint(endPoint);

            if (dist < closestDistance)
            {
                closestRoad = nearbyRoad;
                closestS = start ? 0 : nearbyRoad->Length();
                closestDistance = dist;
                targetScenePos.setX(x);
                targetScenePos.setY(y);
            }
        }
    }


    if (closestRoad != nullptr)
    {
        QVector2D cursorViewPos(viewPos);
        QVector2D targetViewPos(mapFromScene(targetScenePos));
        if (targetViewPos.distanceToPoint(cursorViewPos) < CustomCursorItem::SnapRadiusPx)
        {
            g_PointerRoadS = closestS;
            g_PointerRoad = closestRoad;
        }
    }
}

void MapView::mousePressEvent(QMouseEvent* evt)
{
    QGraphicsView::mousePressEvent(evt);
    SnapCursor(evt->pos());
    if (editMode != Mode_None)
    {
        if (!drawingSession->Update(evt))
        {
            confirmEdit();
        }
    }
}

void MapView::mouseMoveEvent(QMouseEvent* evt)
{
    QGraphicsView::mouseMoveEvent(evt);
    SnapCursor(evt->pos());
    if (editMode != Mode_None)
    {
        drawingSession->Update(evt);
    }
}

void MapView::keyPressEvent(QKeyEvent* evt)
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

void MapView::drawForeground(QPainter* painter, const QRectF& rect)
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

    // ROADRUNNERTODO: draw cursor

    viewport()->update();
}

void MapView::AdjustSceneRect()
{
    auto original = scene()->itemsBoundingRect();
    QRectF paded(original.left() - ViewPadding, original.top() - ViewPadding,
        original.width() + 2 * ViewPadding, original.height() + 2 * ViewPadding);
    setSceneRect(paded);
}

void MapView::confirmEdit()
{
    drawingSession->Complete();
    quitEdit();

    AdjustSceneRect();
}

void MapView::quitEdit()
{
    SetEditMode(editMode);
}