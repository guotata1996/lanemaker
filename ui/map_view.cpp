#include "map_view.h"
#include "main_widget.h"
#include "road_drawing.h"
#include "road_graphics.h"
#include "change_tracker.h"
#include "junction.h"
#include "action_manager.h"
#include "constants.h"
#include "vehicle.h"
#include "spatial_indexer.h"

#include <sstream>
#include <iomanip> //std::setprecision
#include <qscrollbar.h>
#include <qmessagebox.h>
#include <qapplication.h>


std::weak_ptr<RoadRunner::Road> g_PointerRoad;
double g_PointerRoadS; /*Continous between 0 and Length() if g_PointerRoad is valid*/
int g_PointerLane;

MapView* g_mapView;
extern RoadRunner::LanePlan leftProfileSetting, rightProfileSetting;

std::weak_ptr<Vehicle> g_PointerVehicle;

QString PointerObjectInfo()
{
    if (!g_PointerVehicle.expired())
    {
        auto vehicleInfo = QString("Car %1").arg(g_PointerVehicle.lock()->ID.c_str());
        return vehicleInfo;
    }

    else if (!g_PointerRoad.expired())
    {
        auto roadInfo = QString("Road %1 @%2 Lane %3")
            .arg(g_PointerRoad.lock()->ID().c_str())
            .arg(g_PointerRoadS, 6, 'f', 3)
            .arg(g_PointerLane);

        auto roadElevation = g_PointerRoad.lock()->generated.ref_line.elevation_profile.get(g_PointerRoadS);
        if (roadElevation != 0)
        {
            roadInfo += QString(" Z %1").arg(roadElevation, 5, 'f', 2);
        }
        return roadInfo;
    }
    return QString();
}

MapView::MapView(MainWidget* v, QGraphicsScene* scene) :
    QGraphicsView(scene), parentContainer(v)
{
    ResetSceneRect();
    g_mapView = this;
}

void MapView::ResetSceneRect()
{
    setSceneRect(-ViewPadding, -ViewPadding, 2 * ViewPadding, 2 * ViewPadding);
    if (backgroundItem != nullptr)
    {
        scene()->removeItem(backgroundItem);
        backgroundItem = nullptr;
    }
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

void MapView::scrollContentsBy(int dx, int dy)
{
    QGraphicsView::scrollContentsBy(dx, dy);
    parentContainer->RecordViewTransform();
}

void MapView::SetEditMode(RoadRunner::EditMode aMode)
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

void MapView::SetBackground(const QPixmap& image)
{
    if (backgroundItem != nullptr)
    {
        scene()->removeItem(backgroundItem);
    }
    backgroundItem = new QGraphicsPixmapItem(image);
    QTransform trans;
    trans.scale(1, -1);
    trans.translate(-image.width() / 2, -image.height() / 2);
    backgroundItem->setTransform(trans);
    backgroundItem->setZValue(-999);
    scene()->addItem(backgroundItem);

    AdjustSceneRect();
}

void MapView::OnMousePress(const RoadRunner::MouseAction& evt)
{
    if (editMode != RoadRunner::Mode_None)
    {
        if (!drawingSession->Update(evt))
        {
            confirmEdit();
        }
    }
    //if (evt.button == Qt::MiddleButton)
    //{
    //    auto screenPos = mapFromScene(evt.sceneX, evt.sceneY);
    //    prevDragMousePos.emplace(screenPos);
    //}
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

void MapView::OnMouseDoubleClick(const RoadRunner::MouseAction& evt)
{
    if (editMode != RoadRunner::Mode_None)
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

void MapView::OnMouseMove(const RoadRunner::MouseAction& evt)
{
    SnapCursor(QPointF(evt.screenX, evt.screenY));
    if (editMode != RoadRunner::Mode_None)
    {
        drawingSession->Update(evt);
    }
    if (evt.button == Qt::MiddleButton)
    {
        auto screenPos = mapFromScene(evt.screenX, evt.screenY);
        auto offset = prevDragMousePos.value() - screenPos;
        verticalScrollBar()->setValue(verticalScrollBar()->value() + offset.y());
        horizontalScrollBar()->setValue(horizontalScrollBar()->value() + offset.x());

        prevDragMousePos.emplace(screenPos);
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

void MapView::OnMouseRelease(const RoadRunner::MouseAction& evt)
{
    if (editMode != RoadRunner::Mode_None)
    {
        drawingSession->Update(evt);
    }

    if (evt.button == Qt::MiddleButton)
    {
        prevDragMousePos.reset();
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

void MapView::OnKeyPress(const RoadRunner::KeyPressAction& evt)
{
    switch (evt.key)
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
        auto g_vehicle = g_PointerVehicle.lock();
        if (g_vehicle != nullptr)
        {
            spdlog::info(g_vehicle->Log());
        }
        else if (g_road != nullptr)
        {
            std::stringstream ss;
            ss << "Road" << g_road->ID() << ": Length= " << std::setprecision(3) << g_road->Length();
            if (g_road->generated.junction != "-1")
            {
                ss << " is a connecting road of:\n";
                auto junc = static_cast<RoadRunner::Junction*>(IDGenerator::ForJunction()->GetByID(g_road->generated.junction));
                ss << junc->Log();
            }
            else
            {
                ss << g_road->generated.rr_profile.ToString();
                ss << g_road->RefLine().elevation_profile.ToString();
                if (!g_road->generated.boundaryHide.empty())
                {
                    ss << "\n====Boundary Hide====\n";
                    for (const auto& bh_length : g_road->generated.boundaryHide)
                    {
                        ss << "Boundary hide at" <<
                            (bh_length.first.first == odr::RoadLink::ContactPoint_Start ? "start" : "end")
                            << " side " << bh_length.first.second << " = " << bh_length.second << '\n';
                    }
                }
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
    }
}

void MapView::keyPressEvent(QKeyEvent* evt)
{
    QGraphicsView::keyPressEvent(evt);
    
    RoadRunner::ActionManager::Instance()->Record(evt);
    try
    {
        OnKeyPress(RoadRunner::KeyPressAction(evt));
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

void MapView::PostEditActions()
{
    g_PointerRoad.reset();

    AdjustSceneRect();
}

void MapView::confirmEdit()
{
    RoadRunner::ChangeTracker::Instance()->StartRecordEdit();
    bool cleanState = drawingSession->Complete();
    RoadRunner::ChangeTracker::Instance()->FinishRecordEdit(!cleanState);
    quitEdit();

    PostEditActions();
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

void MapView::SnapCursor(const QPointF& scenePos)
{
    if (!g_PointerVehicle.expired())
    {
        g_PointerVehicle.lock()->EnableRouteVisual(false, RoadRunner::ChangeTracker::Instance()->Map());
    }
    g_PointerVehicle.reset();

    // Nearby candidates
    double r = RoadRunner::SnapRadiusPx;
    RoadRunner::RayCastQuery ray{ odr::Vec3D{
        static_cast<double>(scenePos.x()),
        static_cast<double>(scenePos.y()), 50}, odr::Vec3D{0, 0, -1}};
    auto rayCastResult = RoadRunner::SpatialIndexer::Instance()->RayCast(ray);
    
    if (rayCastResult.hit)
    {
        g_PointerRoad = static_cast<RoadRunner::Road*>(IDGenerator::ForRoad()->GetByID(
            rayCastResult.roadID))->shared_from_this();
        g_PointerLane = rayCastResult.lane;
        g_PointerRoadS = rayCastResult.s;
    }
    else
    {
        g_PointerRoad.reset();
    }

    auto cursorInfo = QString("(%1, %2)| ")
        .arg(scenePos.x(), 4, 'f', 1)
        .arg(scenePos.y(), 4, 'f', 1);

    parentContainer->SetHovering(cursorInfo + PointerObjectInfo());
    if (!g_PointerVehicle.expired())
    {
        g_PointerVehicle.lock()->EnableRouteVisual(true, RoadRunner::ChangeTracker::Instance()->Map());
    }
}

void MapView::AdjustSceneRect()
{
    //QRectF original(0, 0, 0, 0);

    //if (backgroundItem != nullptr)
    //{
    //    original = backgroundItem->sceneBoundingRect();
    //}
    //else
    //{

    //    for (auto item : scene()->items())
    //    {
    //        if (dynamic_cast<RoadRunner::LaneGraphics*>(item) != nullptr)
    //        {
    //            original = original.united(item->sceneBoundingRect());
    //        }
    //    }
    //}

    //QRectF paded(original.left() - ViewPadding, original.top() - ViewPadding,
    //    original.width() + 2 * ViewPadding, original.height() + 2 * ViewPadding);
    //setSceneRect(paded);
}
