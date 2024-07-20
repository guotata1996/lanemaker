#include "map_view.h"
#include "main_widget.h"
#include "road_drawing.h"
#include "road_graphics.h"
#include "change_tracker.h"
#include "junction.h"
#include "action_manager.h"

#include <sstream>
#include <iomanip> //std::setprecision
#include <qscrollbar.h>
#include <qmessagebox.h>
#include <qapplication.h>


std::weak_ptr<RoadRunner::Road> g_PointerRoad;
double g_PointerRoadS; /*Continous between 0 and Length() if g_PointerRoad is valid*/
int g_PointerLane;

std::vector<std::pair<RoadRunner::LaneGraphics*, double>> rotatingRoads;
int rotatingIndex;

MapView* g_mapView;
extern RoadRunner::SectionProfile leftProfileSetting, rightProfileSetting;

MapView::MapView(MainWidget* v) :
    QGraphicsView(), parentContainer(v)
{
    ResetSceneRect();
    g_mapView = this;
}

void MapView::ResetSceneRect()
{
    setSceneRect(-ViewPadding, -ViewPadding, 2 * ViewPadding, 2 * ViewPadding);
}

double MapView::Zoom() const
{
    return std::sqrt(transform().m11() * transform().m11() + transform().m12() * transform().m12());
}

void MapView::SetViewFromReplay(double zoomSliderVal, double rotateSliderVal,
    int hScrollbar, int vScrollbar)
{
    parentContainer->SetViewFromReplay(zoomSliderVal, rotateSliderVal);
    horizontalScrollBar()->setValue(hScrollbar);
    verticalScrollBar()->setValue(vScrollbar);
}

void MapView::showScale()
{
    showingScale = true;
}

void MapView::hideScale()
{
    showingScale = false;
}

#if QT_CONFIG(wheelevent)
void MapView::wheelEvent(QWheelEvent* e)
{
    if (e->modifiers() & Qt::ControlModifier) {
        if (e->angleDelta().y() > 0)
            parentContainer->zoomInBy(6);
        else
            parentContainer->zoomOutBy(6);
        e->accept();
    }
    else {
        QGraphicsView::wheelEvent(e);
    }
}
#endif

void MapView::scrollContentsBy(int dx, int dy)
{
    QGraphicsView::scrollContentsBy(dx, dy);
    parentContainer->RecordViewTransform();
}

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
    case Mode_CreateLanes:
        drawingSession = new LanesCreationSession(this);
        break;
    case Mode_Destroy:
        drawingSession = new RoadDestroySession(this);
        break;
    case Mode_Modify:
        drawingSession = new RoadModificationSession(this);
        break;
    default:
        break;
    }
}

void MapView::OnMousePress(QMouseEvent* evt)
{
    if (editMode != Mode_None)
    {
        if (!drawingSession->Update(evt))
        {
            confirmEdit();
        }
    }
}

void MapView::mousePressEvent(QMouseEvent* evt)
{
    QGraphicsView::mousePressEvent(evt);
    
    RoadRunner::ActionManager::Instance()->Record(evt);
    try
    {
        OnMousePress(evt);
    }
    catch (std::exception e)
    {
        handleException(e);
    }
}

void MapView::OnMouseDoubleClick(QMouseEvent* evt)
{
    if (editMode != Mode_None)
    {
        drawingSession->Update(evt);
    }
}

void MapView::mouseDoubleClickEvent(QMouseEvent* evt)
{
    QGraphicsView::mouseDoubleClickEvent(evt);
    
    RoadRunner::ActionManager::Instance()->Record(evt);
    try
    {
        OnMouseDoubleClick(evt);
    }
    catch (std::exception e)
    {
        handleException(e);
    }
}

void MapView::OnMouseMove(QMouseEvent* evt)
{
    SnapCursor(evt->pos());
    if (editMode != Mode_None)
    {
        drawingSession->Update(evt);
    }
}

void MapView::mouseMoveEvent(QMouseEvent* evt)
{
    QGraphicsView::mouseMoveEvent(evt);
    
    RoadRunner::ActionManager::Instance()->Record(evt);
    try
    {
        OnMouseMove(evt);
    }
    catch (std::exception e)
    {
        handleException(e);
    }
}

void MapView::OnMouseRelease(QMouseEvent* evt)
{
    if (editMode != Mode_None)
    {
        drawingSession->Update(evt);
    }
}

void MapView::mouseReleaseEvent(QMouseEvent* evt)
{
    QGraphicsView::mouseReleaseEvent(evt);
    
    RoadRunner::ActionManager::Instance()->Record(evt);
    try
    {
        OnMouseRelease(evt);
    }
    catch (std::exception e)
    {
        handleException(e);
    }
}

void MapView::OnKeyPress(QKeyEvent* evt)
{
    switch (evt->key())
    {
    case Qt::Key_Escape:
        quitEdit();
        break;
    case Qt::Key_Return:
        if (drawingSession != nullptr)
            confirmEdit();
        break;
    case Qt::Key_I:
    {
        auto g_road = g_PointerRoad.lock();
        if (g_road != nullptr)
        {
            std::stringstream ss;
            ss << "Road" << g_road->ID() << ": Length= " << std::setprecision(3) << g_road->Length();
            if (g_road->generated.junction != "-1")
            {
                ss << " is connecting in junction " << g_road->generated.junction;
                auto junc = static_cast<RoadRunner::Junction*>(IDGenerator::ForJunction()->GetByID(g_road->generated.junction));
                ss << junc->Log();
            }
            else
            {
                ss << g_road->generated.rr_profile.Log();
            }

            if (g_road->predecessorJunction != nullptr)
            {
                ss << "\nPred junction:" << g_road->predecessorJunction->Log();
            }
            if (g_road->successorJunction != nullptr)
            {
                ss << "\nSucc junction:" << g_road->successorJunction->Log();
            }

            spdlog::info("{}", ss.str());
        }
        else
        {
            spdlog::info("NonConnRoad={}, NRoadID={}, JuctionID={}, N visible graphics items={}",
                World::Instance()->allRoads.size(),
                IDGenerator::ForRoad()->size(),
                IDGenerator::ForJunction()->size(),
                scene()->items(mapToScene(viewport()->geometry())).size());
        }
        break;
    }
    case Qt::Key_A:
        if (!rotatingRoads.empty())
        {
            rotatingIndex = (rotatingIndex + 1) % rotatingRoads.size();
            g_PointerRoad = rotatingRoads[rotatingIndex].first->Road();
            g_PointerRoadS = rotatingRoads[rotatingIndex].second;
            if (drawingSession != nullptr)
            {
                drawingSession->SetHighlightTo(g_PointerRoad.lock());
            }
        }
        break;
    }
}

void MapView::keyPressEvent(QKeyEvent* evt)
{
    QGraphicsView::keyPressEvent(evt);
    
    RoadRunner::ActionManager::Instance()->Record(evt);
    try
    {
        OnKeyPress(evt);
    }
    catch (std::exception e)
    {
        handleException(e);
    }
}

void MapView::paintEvent(QPaintEvent* evt)
{
    QGraphicsView::paintEvent(evt);
    parentContainer->Painted();
    if (showingScale)
    {
        // Force update foreground
        viewport()->update();
    }
}

void MapView::drawForeground(QPainter* painter, const QRectF& rect)
{
    Q_UNUSED(rect);

    QGraphicsView::drawForeground(painter, rect);
    if (showingScale)
    {
        painter->save();

        QRectF rt = viewport()->rect();
        painter->setWorldMatrixEnabled(false);

        painter->setBrush(Qt::NoBrush);
        painter->setPen(QPen(Qt::black, 3));

        auto probeA = mapToScene(QPoint(0, 0));
        auto probeB = mapToScene(QPoint(100, 0));
        auto probeLength = QVector2D(probeA).distanceToPoint(QVector2D(probeB));
        float targetLength = 50;
        int pixelLength = 100 / probeLength * targetLength;
        if (pixelLength * 2 > rt.width() - 30)
        {
            // Use smaller target length for large zoom 
            targetLength /= 5;
            pixelLength /= 5;
        }

        QPoint origin(30, rt.bottom() - 30);
        QPoint cursor1(origin.x() + pixelLength, origin.y());
        QPoint cursor1Bar(cursor1.x(), cursor1.y() - 10);

        QPoint cursor2(cursor1.x() + pixelLength, origin.y());
        QPoint cursor2Bar(cursor2.x(), cursor2.y() - 10);

        painter->drawLine(origin, cursor2);
        painter->drawLine(cursor1, cursor1Bar);
        painter->drawLine(cursor2, cursor2Bar);

        QRectF cursor1Box(cursor1.x() - 5, cursor1.y(), 30, 20);
        painter->drawText(cursor1Box, QString("%1").arg(targetLength));
        QRectF cursor2Box(cursor2.x() - 15, cursor2.y(), 35, 20);
        painter->drawText(cursor2Box, QString("%1m").arg(targetLength * 2));

        painter->restore();
    }
}

void MapView::AdjustSceneRect()
{
    //auto original = scene()->itemsBoundingRect();
    QRectF original(0, 0, 0, 0);
    for (auto item : scene()->items())
    {
        if (dynamic_cast<RoadRunner::LaneGraphics*>(item) != nullptr)
        {
            original = original.united(item->sceneBoundingRect());
        }
    }

    QRectF paded(original.left() - ViewPadding, original.top() - ViewPadding,
        original.width() + 2 * ViewPadding, original.height() + 2 * ViewPadding);
    setSceneRect(paded);
}

void MapView::confirmEdit()
{
    RoadRunner::ChangeTracker::Instance()->StartRecordEdit();
    bool cleanState = drawingSession->Complete();
    RoadRunner::ChangeTracker::Instance()->FinishRecordEdit(!cleanState);
    quitEdit();

    AdjustSceneRect();
}

void MapView::quitEdit()
{
    SetEditMode(editMode);
}

void MapView::handleException(std::exception e)
{
    RoadRunner::ActionManager::Instance()->MarkException();
    auto msg = std::string(e.what()) + "\nReplayable at " + RoadRunner::ActionManager::Instance()->AutosavePath();
    auto quit = QMessageBox::question(this, "Quit now?", 
        QString::fromStdString(msg), QMessageBox::Yes | QMessageBox::No);
    if (quit == QMessageBox::Yes)
    {
        QCoreApplication::quit();
    }
}

void MapView::SnapCursor(const QPoint& viewPos)
{
    QVector2D viewPosVec(viewPos);
    rotatingRoads.clear();

    // Direct candidates
    decltype(rotatingRoads) directOver, indirectOver;

    // Nearby candidates
    double r = CustomCursorItem::SnapRadiusPx;
    auto candidates = items(viewPos.x() - r, viewPos.y() - r, 2 * r, 2 * r);
    for (auto item : candidates)
    {
        while (item != nullptr)
        {
            auto laneGraphicsItem = dynamic_cast<RoadRunner::LaneGraphics*>(item);
            if (laneGraphicsItem != nullptr)
            {
                double s;
                auto snapResult = laneGraphicsItem->SnapCursor(mapToScene(viewPos), s);
                if (snapResult.expired())
                {
                    double x, y;
                    laneGraphicsItem->Road()->GetEndPoint(true, x, y);
                    QVector2D startViewPos(mapFromScene(QPoint(x, y)));
                    double distToStart = startViewPos.distanceToPoint(viewPosVec);

                    laneGraphicsItem->Road()->GetEndPoint(false, x, y);
                    QVector2D endViewPos(mapFromScene(QPoint(x, y)));
                    double distToEnd = endViewPos.distanceToPoint(viewPosVec);

                    if (std::min(distToStart, distToEnd) < CustomCursorItem::SnapRadiusPx)
                    {
                        double closestS = distToStart < distToEnd ? 0 : laneGraphicsItem->Road()->Length();
                        indirectOver.push_back(std::make_pair(laneGraphicsItem, closestS));
                    }
                }
                else
                {
                    directOver.push_back(std::make_pair(laneGraphicsItem, s));
                }

                break;
            }
            item = item->parentItem();
        }
    }

    rotatingIndex = 0;
    if (!directOver.empty())
    {
        // Try retain previous selected
        for (int i = 0; i != directOver.size(); ++i)
        {
            if (directOver[i].first->Road() == g_PointerRoad.lock())
            {
                rotatingIndex = i;
                break;
            }
        }

        rotatingRoads = directOver;
    }
    else if (!indirectOver.empty())
    {
        // Try retain previous selected
        for (int i = 0; i != indirectOver.size(); ++i)
        {
            if (indirectOver[i].first->Road() == g_PointerRoad.lock())
            {
                rotatingIndex = i;
                break;
            }
        }
        if (rotatingIndex == -1)
        {
            std::sort(indirectOver.begin(), indirectOver.end(),
                [this, viewPosVec](const auto& a, const auto& b) {
                    odr::Vec2D p1 = a.first->Road()->RefLine().get_xy(a.second);
                    QVector2D q1(mapFromScene(p1[0], p1[1]));
                    odr::Vec2D p2 = b.first->Road()->RefLine().get_xy(b.second);
                    QVector2D q2(mapFromScene(p1[0], p2[1]));
                    return q1.distanceToPoint(viewPosVec) < q2.distanceToPoint(viewPosVec);
                });
        }

        rotatingRoads = indirectOver;
    }
    else
    {
        rotatingRoads.clear();
    }

    QString txt;
    if (rotatingRoads.empty())
    {
        g_PointerRoad.reset();
    }
    else
    {
        g_PointerRoad = rotatingRoads[rotatingIndex].first->Road();
        g_PointerLane = rotatingRoads[rotatingIndex].first->LaneID();
        g_PointerRoadS = rotatingRoads[rotatingIndex].second;
        std::stringstream ss;
        ss << "Road " << g_PointerRoad.lock()->ID() << " @ " << g_PointerRoadS << " Lane " << g_PointerLane;
        txt = QString::fromStdString(ss.str());
    }

    parentContainer->SetHovering(txt);
}
