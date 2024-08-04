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

#include "curve_fitting.h"


std::weak_ptr<RoadRunner::Road> g_PointerRoad;
double g_PointerRoadS; /*Continous between 0 and Length() if g_PointerRoad is valid*/
int g_PointerLane;

std::vector<std::pair<RoadRunner::LaneGraphics*, double>> rotatingRoads;
int rotatingIndex;

MapView* g_mapView;
extern RoadRunner::SectionProfile leftProfileSetting, rightProfileSetting;

MapView::MapView(MainWidget* v, QGraphicsScene* scene) :
    QGraphicsView(scene), parentContainer(v)
{
    ResetSceneRect();
    g_mapView = this;

    const double degToRad = M_PI / 180;
    const odr::Vec2D startPos{ 0, 0 };
    const odr::Vec2D startHdg{ 1, 0 };
    auto pen = QPen(QBrush(Qt::black), 0.02);
    const double radius = 10;

    std::map<int, std::pair<int, int>> posAngleCombo =
    {
        {5, {5, 60}},
        {10, {5, 70}},
        {15, {5, 110}},
        {20, {5, 130}},
        {25, {5, 130}},
        {30, {5, 130}},
        {35, {5, 145}},
        {40, {5, 140}},
        {45, {5, 145}},
        {50, {5, 140}},
        {55, {5, 140}},
        {60, {5, 140}},
        {65, {5, 140}},
        {70, {5, 135}},
        {75, {5, 140}},
        {80, {5, 140}},
        {85, {5, 140}},
        {90, {5, 140}},
        {95, {5, 140}},
        {100, {5, 140}},
        {105, {5, 140}},
        {110, {5, 140}},
        {115, {5, 140}},
        {120, {5, 145}},
        {125, {5, 145}},
        {130, {5, 145}},
        {135, {10, 145}},
        {140, {20, 145}},
        {145, {35, 145}},
        {150, {40, 145}},
        {155, {50, 145}},
        {160, {60, 95}},
    };
    
    for (auto posAndAngle: posAngleCombo)
    {
        int posAngle = posAndAngle.first;
        odr::Vec2D endPos{ std::cos(posAngle * degToRad), std::sin(posAngle * degToRad) };
        endPos = odr::mut(radius, endPos);

        int hdgBegin = posAndAngle.second.first;
        int hdgEnd = posAndAngle.second.second;

        for (int hdg = hdgBegin; hdg <= hdgEnd; hdg += 5)
        {
            int hdgAngle = (posAngle + hdg);
            odr::Vec2D endHdg{ std::cos(hdgAngle * degToRad), std::sin(hdgAngle * degToRad) };
            auto fitGeo = RoadRunner::FitSpiral(startPos, startHdg, endPos, endHdg);
            if (fitGeo == nullptr)
            {
                spdlog::error("No ans given angle {} hdg {}", posAngle, hdg);
                continue;
            }

            // Draw
            /*
            double prevS = -1;
            for (double s = 0; s < fitGeo->length; s += 0.05)
            {
                if (prevS >= 0)
                {
                    auto p0 = fitGeo->get_xy(prevS);
                    auto p1 = fitGeo->get_xy(s);
                    QLineF seg(QPointF(p0[0], p0[1]), QPointF(p1[0], p1[1]));
                    scene->addLine(seg, pen);
                }
                prevS = s;
            }*/
        }
    }
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
        drawingSession = new RoadCreationSession(this);
        break;
    case RoadRunner::Mode_CreateLanes:
        drawingSession = new LanesCreationSession(this);
        break;
    case RoadRunner::Mode_Destroy:
        drawingSession = new RoadDestroySession(this);
        break;
    case RoadRunner::Mode_Modify:
        drawingSession = new RoadModificationSession(this);
        break;
    default:
        break;
    }
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
}

void MapView::mousePressEvent(QMouseEvent* evt)
{
    QGraphicsView::mousePressEvent(evt);

    spdlog::info("Mouse event at {},{}", evt->pos().x(), evt->pos().y());
    
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
    auto viewPos = mapFromScene(evt.sceneX, evt.sceneY);
    SnapCursor(viewPos);
    if (editMode != RoadRunner::Mode_None)
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

void MapView::OnMouseRelease(const RoadRunner::MouseAction& evt)
{
    if (editMode != RoadRunner::Mode_None)
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
        if (g_road != nullptr)
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
            g_PointerRoad = rotatingRoads[rotatingIndex].first->GetRoad();
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
        OnKeyPress(RoadRunner::KeyPressAction{evt->key()});
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
    auto scenePos = mapToScene(viewPos);
    for (auto item : candidates)
    {
        while (item != nullptr)
        {
            auto laneGraphicsItem = dynamic_cast<RoadRunner::LaneGraphics*>(item);
            if (laneGraphicsItem != nullptr)
            {
                double s;
                auto snapResult = laneGraphicsItem->SnapCursor(scenePos, s);
                if (snapResult.expired())
                {
                    double x, y;
                    laneGraphicsItem->GetRoad()->GetEndPoint(true, x, y);
                    QVector2D startViewPos(mapFromScene(QPoint(x, y)));
                    double distToStart = startViewPos.distanceToPoint(viewPosVec);

                    laneGraphicsItem->GetRoad()->GetEndPoint(false, x, y);
                    QVector2D endViewPos(mapFromScene(QPoint(x, y)));
                    double distToEnd = endViewPos.distanceToPoint(viewPosVec);

                    if (std::min(distToStart, distToEnd) < CustomCursorItem::SnapRadiusPx)
                    {
                        double closestS = distToStart < distToEnd ? 0 : laneGraphicsItem->GetRoad()->Length();
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
            if (directOver[i].first->GetRoad() == g_PointerRoad.lock())
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
            if (indirectOver[i].first->GetRoad() == g_PointerRoad.lock())
            {
                rotatingIndex = i;
                break;
            }
        }
        if (rotatingIndex == -1)
        {
            std::sort(indirectOver.begin(), indirectOver.end(),
                [this, viewPosVec](const auto& a, const auto& b) {
                    odr::Vec2D p1 = a.first->GetRoad()->RefLine().get_xy(a.second);
                    QVector2D q1(mapFromScene(p1[0], p1[1]));
                    odr::Vec2D p2 = b.first->GetRoad()->RefLine().get_xy(b.second);
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

    auto cursorInfo = QString("(%1, %2)")
        .arg(scenePos.x(), 4, 'f', 1)
        .arg(scenePos.y(), 4, 'f', 1);
    QString roadInfo;
    if (rotatingRoads.empty())
    {
        g_PointerRoad.reset();
    }
    else
    {
        g_PointerRoad = rotatingRoads[rotatingIndex].first->GetRoad();
        g_PointerLane = rotatingRoads[rotatingIndex].first->LaneID();
        g_PointerRoadS = rotatingRoads[rotatingIndex].second;
        
        roadInfo = QString(" | Road %1 @%2 Lane %3")
            .arg(g_PointerRoad.lock()->ID().c_str())
            .arg(g_PointerRoadS, 6, 'f', 3)
            .arg(g_PointerLane);
    }

    parentContainer->SetHovering(cursorInfo + roadInfo);
}
