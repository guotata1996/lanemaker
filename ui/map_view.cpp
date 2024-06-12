#include "map_view.h"
#include "main_widget.h"
#include "road_drawing.h"
#include "road_graphics.h"
#include "change_tracker.h"
#include "junction.h"

#include <sstream>
#include <iomanip> //std::setprecision
#include <qevent.h>

std::weak_ptr<RoadRunner::Road> g_PointerRoad;
double g_PointerRoadS; /*Continous between 0 and Length() if g_PointerRoad is valid*/
int g_PointerLane;

std::vector<std::pair<RoadRunner::LaneGraphics*, double>> rotatingRoads;
int rotatingIndex;

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

    view->SetHovering(txt);
}

void MapView::mousePressEvent(QMouseEvent* evt)
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

void MapView::mouseDoubleClickEvent(QMouseEvent* evt)
{
    if (editMode != Mode_None)
    {
        drawingSession->Update(evt);
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

void MapView::mouseReleaseEvent(QMouseEvent* evt)
{
    QGraphicsView::mouseReleaseEvent(evt);
    if (editMode != Mode_None)
    {
        drawingSession->Update(evt);
    }
}

void MapView::keyPressEvent(QKeyEvent* evt)
{
    QGraphicsView::keyPressEvent(evt);
    switch (evt->key())
    {
    case Qt::Key_Escape:
        quitEdit();
        break;
    case Qt::Key_Return:
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

void MapView::paintEvent(QPaintEvent* evt)
{
    QGraphicsView::paintEvent(evt);
    view->Painted();
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