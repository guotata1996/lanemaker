#include "map_view_gl.h"
#include <QMouseEvent>
#include <QKeyEvent>
#include <QCoreApplication>

#include "main_widget.h"
#include "id_generator.h"
#include "spatial_indexer.h"
#include "triangulation.h"
#include "action_manager.h"
#include "constants.h"
#include "change_tracker.h"
#include "vehicle.h"
#include "junction.h"

#include <spdlog/spdlog.h>

namespace LM
{
    MapViewGL* g_mapViewGL;
    std::string g_PointerRoadID;
    double g_PointerRoadS;
    int g_PointerLane;
    odr::Vec2D g_PointerOnGround;
    odr::Vec3D g_CameraPosition;
    int g_createRoadElevationOption;
    unsigned int g_PointerVehicle;
    bool touchScreen;

    MapViewGL::MapViewGL() :
        permanentBuffer(std::make_unique<GLBufferManage>(MaxRoadVertices)), 
        temporaryBuffer(std::make_unique<GLBufferManage>(MaxTemporaryVertices)),
        backgroundBuffer(std::make_unique<GLBufferManage>(1 << 12)),
        vehicleBuffer{
            GLBufferManageInstanced(":/models/jeep.obj", ":/models/jeep.jpg", MaxInstancesPerType),
            GLBufferManageInstanced(":/models/cadillac.obj", ":/models/cadillac.jpg", MaxInstancesPerType),
            GLBufferManageInstanced(":/models/military.obj", ":/models/military.jpg", MaxInstancesPerType)
        }
    {
        g_mapViewGL = this;
        g_createRoadElevationOption = 0;
        ResetCamera();
        touchScreen = false;
        setAttribute(Qt::WA_AcceptTouchEvents);
    }

    void MapViewGL::CleanupResources()
    {
        makeCurrent();
        permanentBuffer->CleanupResources();
        temporaryBuffer->CleanupResources();
        backgroundBuffer->CleanupResources();
        for (auto& buffer : vehicleBuffer)
        {
            buffer.CleanupResources();
        }
    }

    void MapViewGL::ResetCamera()
    {
        m_camera.setTranslation(0, -200, 250);
        m_camera.setRotation(30, QVector3D(1, 0, 0));
    }

    unsigned int MapViewGL::AddQuads(const odr::Line3D& lBorder, const odr::Line3D& rBorder, QColor color, unsigned int objID)
    {
        bool temporary = objID == -1;
        auto gid = IDGenerator::ForType(temporary ? IDType::Graphics_Temporary : IDType::Graphics)->GenerateID(this);
        bool success = true;
        if (temporary)
        {
            success = temporaryBuffer->AddQuads(gid, objID, lBorder, rBorder, color);
        }
        else
        {
            success = permanentBuffer->AddQuads(gid, objID, lBorder, rBorder, color);
        }
        if (!success)
        {
            throw std::logic_error("Graphics buffer out of space!");
        }
        return gid;
    }

    void MapViewGL::LineToQuads(const odr::Line3D& border, double width, odr::Line3D& lBorder, odr::Line3D& rBorder)
    {
        lBorder.reserve(border.size());
        rBorder.reserve(border.size());

        for (int i = 0; i != border.size(); ++i)
        {
            odr::Vec3D tangent;
            if (i == 0)
            {
                tangent = odr::sub(border[i + 1], border[i]);
            }
            else if (i == border.size() - 1)
            {
                tangent = odr::sub(border[i], border[i - 1]);
            }
            else
            {
                auto tan1 = odr::normalize(odr::sub(border[i + 1], border[i]));
                auto tan2 = odr::normalize(odr::sub(border[i], border[i - 1]));
                tangent = odr::add(tan1, tan2);
            }
            if (odr::squaredNorm(tangent) == 0)
                continue;
            tangent = odr::normalize(tangent);

            odr::Vec3D radio{ -tangent[1], tangent[0], 0 };
            lBorder.push_back(odr::add(border[i], odr::mut(width / 2, radio)));
            rBorder.push_back(odr::add(border[i], odr::mut(-width / 2, radio)));
        }
    }

    unsigned int MapViewGL::AddLine(const odr::Line3D& border, double width, QColor color, unsigned int objID)
    {
        bool temporary = objID == -1;
        odr::Line3D lBorder, rBorder;
        LineToQuads(border, width, lBorder, rBorder);
        return AddQuads(lBorder, rBorder, color, objID);
    }

    unsigned int MapViewGL::AddPoly(const odr::Line3D& boundary, QColor color, unsigned int objID)
    {
        bool temporary = objID == -1;
        auto gid = IDGenerator::ForType(temporary ? IDType::Graphics_Temporary : IDType::Graphics)->GenerateID(this);
        bool success = true;
        if (temporary)
        {
            success = temporaryBuffer->AddPoly(gid, objID, boundary, color);
        }
        else
        {
            success = permanentBuffer->AddPoly(gid, objID, boundary, color);
        }
        if (!success)
        {
            throw std::logic_error("Graphics buffer out of space!");
        }
        return gid;
    }

    unsigned int MapViewGL::AddColumn(const odr::Line3D& boundary, double h, QColor color, unsigned int objID)
    {
        bool temporary = objID == -1;
        auto gid = IDGenerator::ForType(temporary ? IDType::Graphics_Temporary : IDType::Graphics)->GenerateID(this);
        bool success = true;
        if (temporary)
        {
            success = temporaryBuffer->AddColumn(gid, objID, boundary, h, color);
        }
        else
        {
            success = permanentBuffer->AddColumn(gid, objID, boundary, h, color);
        }
        if (!success)
        {
            throw std::logic_error("Graphics buffer out of space!");
        }
        return gid;
    }

    void MapViewGL::AddInstance(unsigned int id, QColor color, unsigned int variation)
    {
        vehicleBuffer[variation].AddInstance(id, QMatrix4x4(), color);
    }

    void MapViewGL::UpdateObject(unsigned int id, uint8_t flag)
    {
        permanentBuffer->UpdateItem(id, flag);
    }

    uint8_t MapViewGL::GetObjectFlag(unsigned int objectID)
    {
        return permanentBuffer->GetItemFlag(objectID);
    }

    void MapViewGL::UpdateObjectID(unsigned int graphicsID, unsigned int objectID)
    {
        permanentBuffer->UpdateObjectID(graphicsID, objectID);
    }

    void MapViewGL::RemoveItem(unsigned int id, bool temporary)
    {
        if (!temporary)
        {
            permanentBuffer->RemoveItem(id);
        }
        else
        {
            temporaryBuffer->RemoveItem(id);
        }
        IDGenerator::ForType(temporary ? IDType::Graphics_Temporary : IDType::Graphics)->FreeID(id);
    }

    void MapViewGL::RemoveObject(unsigned int objectID)
    {
        permanentBuffer->RemoveObject(objectID);
    }

    void MapViewGL::UpdateInstance(unsigned int id, const QMatrix4x4 trans, unsigned int variation)
    {
        vehicleBuffer[variation].UpdateInstance(id, trans);
    }

    void MapViewGL::RemoveInstance(unsigned int id, unsigned int variation)
    {
        vehicleBuffer[variation].RemoveInstance(id);
    }

    unsigned int MapViewGL::AddBackgroundLine(const odr::Line3D& line, double width, QColor color)
    {
        odr::Line3D lBorder, rBorder;
        LineToQuads(line, width, lBorder, rBorder);
        auto gid = IDGenerator::ForType(IDType::Graphics_Temporary)->GenerateID(this);
        backgroundBuffer->AddQuads(gid, 0, lBorder, rBorder, color);
        return gid;
    }

    void MapViewGL::RemoveBackground(unsigned int gid)
    {
        backgroundBuffer->RemoveItem(gid);
    }

    void MapViewGL::AddSceneLayover(uint32_t id, odr::Vec3D scenePos, QPixmap icon, QRect ltwh, int syntax)
    {
		sceneTiedLayovers.emplace(id, SceneTiedLayover{ id, scenePos, ltwh, icon, syntax });
        update();
    }

    void MapViewGL::RemoveSceneLayover(uint32_t id)
    {
        sceneTiedLayovers.erase(id);
        update();
    }

    int MapViewGL::VBufferUseage_pct() const
    {
        return permanentBuffer->Useage_pct();
    }

    void MapViewGL::initializeGL()
    {
        initializeOpenGLFunctions();

        // draw both sides of faces
        glDisable(GL_CULL_FACE);

        permanentBuffer->Initialize();
        temporaryBuffer->Initialize();
        backgroundBuffer->Initialize();
        for (auto& buff : vehicleBuffer)
        {
            buff.Initialize();
        }
    }

    void MapViewGL::resizeGL(int width, int height)
    {
        m_projection.setToIdentity();
        // create projection matrix, i.e. camera lens
        m_projection.perspective(
            /* vertical angle */ 60.0f,
            /* aspect ratio */   width / float(height),
            /* near */           5.0f,
            /* far */            2000.0f
        );
        // Mind: to not use 0.0 for near plane, otherwise depth buffering and depth testing won't work!
    }

    void MapViewGL::paintGL()
    {
        MainWidget::Instance()->Painted();
        // update cached world2view matrix
        m_worldToView = m_projection * m_camera.toMatrix();// *m_transform.toMatrix();

        const qreal retinaScale = devicePixelRatio(); // needed for Macs with retina display
        glViewport(0, 0, width() * retinaScale, height() * retinaScale);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.1f, 0.15f, 0.3f, 1.0f);

        glDisable(GL_DEPTH_TEST);
        backgroundBuffer->Draw(m_worldToView);
        glEnable(GL_DEPTH_TEST);
        permanentBuffer->Draw(m_worldToView);
        temporaryBuffer->Draw(m_worldToView);
        for (auto& buff : vehicleBuffer)
        {
            buff.Draw(m_worldToView);
        }

        QPainter painter(this);
        
        for (auto& id_btn : sceneTiedLayovers)
        {
            auto& btn = id_btn.second;
            painter.drawPixmap(btn.renderedRect(m_worldToView), btn.icon);
        }
        painter.end();
    }

    void MapViewGL::mousePressEvent(QMouseEvent* event)
    {
        bool ctrlPressed = event->modifiers() & Qt::CTRL;
        if (event->button() == Qt::RightButton && !ctrlPressed)
        {
            lastMousePos = event->pos();
            freeRotateSession = FreeRotController();
            freeRotateSession->Update(QVector2D(lastMousePos), m_camera);
        }
        else if (event->button() == Qt::MiddleButton)
        {
            lastMousePos = event->pos();
            dragPan = true;
        }
        else
        {
            int pressedButton = 0;
            for (auto id_layover : sceneTiedLayovers)
            {
                if (id_layover.second.renderedRect(m_worldToView).contains(event->pos()))
                {
                    pressedButton = id_layover.second.syntax;
                    break;
                }
            }
            if (pressedButton != 0)
            {
                LM::KeyPressAction action(pressedButton);
                LM::ActionManager::Instance()->Record(action);
                emit(KeyPerformedAction(action));
            }
            else
            {
                LM::ActionManager::Instance()->Record(event);
                emit(MousePerformedAction(event));
            }
            ignoreNextMouseRelease = pressedButton != 0;
        }
    }

    void MapViewGL::mouseDoubleClickEvent(QMouseEvent* evt)
    {
        if (evt->button() == Qt::LeftButton)
        {
            LM::ActionManager::Instance()->Record(evt);
            emit(MousePerformedAction(evt));
        }
    }

    void MapViewGL::mouseMoveEvent(QMouseEvent* event)
    {
        UpdateRayHit(event->pos());
        
        bool changeViewPoint = true;
        if (freeRotateSession.has_value())
        {
            if (event->x() < 0 || event->x() > width() ||
                event->y() < 0 || event->y() > height())
            {
                return;
            }

            freeRotateSession->Update(QVector2D(event->pos()), m_camera);
        }
        else if (dragPan)
        {
            auto lastGroundPos = PointerOnGround(lastMousePos);
            m_camera.translate(lastGroundPos - QVector2D(g_PointerOnGround[0], g_PointerOnGround[1]));
        }
        else
        {
            changeViewPoint = false;
        }

        if (changeViewPoint)
        {
            ActionManager::Instance()->Record(m_camera);
        }
        else
        {
            LM::ActionManager::Instance()->Record(event);
            emit(MousePerformedAction(event));
        }
        update();
        lastMousePos = event->pos();
    }

    void MapViewGL::mouseReleaseEvent(QMouseEvent* event)
    {
        if (freeRotateSession.has_value())
        {
            freeRotateSession.reset();
        }
        else if (dragPan)
        {
            dragPan = false;
        }
        else
        {
            if (!ignoreNextMouseRelease)
            {
                LM::ActionManager::Instance()->Record(event);
                emit(MousePerformedAction(event));
            }
        }
    }

    void MapViewGL::wheelEvent(QWheelEvent* event)
    {
        bool ctrlPressed = event->modifiers() & Qt::CTRL;
        auto dir = event->angleDelta().y() > 0 ? 1 : -1;

        if (ctrlPressed)
        {
            g_createRoadElevationOption += dir;
            ActionManager::Instance()->Record(g_createRoadElevationOption);
            emit(MousePerformedAction(event)); // immediately repaint cursor
        }
        else
        {
            auto delta = dir * PointerDirection(lastMousePos) * 10;
            m_camera.translate(delta);
            ActionManager::Instance()->Record(m_camera);
        }

        update();
    }

    void MapViewGL::keyPressEvent(QKeyEvent* event)
    {
        bool changeViewPoint = false;
        
        auto flatForward = m_camera.forward().toVector2D();
        flatForward.normalize();
        flatForward *= 5;
        if (event->key() == Qt::Key_W)
        {
            m_camera.translate(flatForward.x(), flatForward.y(), 0);
            changeViewPoint = true;
        }
        else if (event->key() == Qt::Key_S)
        {
            m_camera.translate(-flatForward.x(), -flatForward.y(), 0);
            changeViewPoint = true;
        }
        else if (event->key() == Qt::Key_A)
        {
            m_camera.translate(-flatForward.y(), flatForward.x(), 0);
            changeViewPoint = true;
        }
        else if (event->key() == Qt::Key_D)
        {
            m_camera.translate(flatForward.y(), -flatForward.x(), 0);
            changeViewPoint = true;
        }
        else if (event->key() == Qt::Key_Q)
        {
            m_camera.rotate(-5, QVector3D(0, 0, 1));
            changeViewPoint = true;
        }
        else if (event->key() == Qt::Key_E)
        {
            m_camera.rotate(5, QVector3D(0, 0, 1));
            changeViewPoint = true;
        }

        if (changeViewPoint)
        {
            LM::ActionManager::Instance()->Record(m_camera);
        }
        else
        {
            LM::ActionManager::Instance()->Record(event);
            emit(KeyPerformedAction(event));
        }
        // update cached world2view matrix
        update();
    }

    bool MapViewGL::event(QEvent* event)
    {
        if (event->type() == QEvent::TouchBegin ||
            event->type() == QEvent::TouchUpdate ||
            event->type() == QEvent::TouchEnd) 
        {
            touchScreen = true;
            QTouchEvent* touchEvent = static_cast<QTouchEvent*>(event);
            const QList<QTouchEvent::TouchPoint>& points = touchEvent->touchPoints();


            if (event->type() == QEvent::TouchEnd)
            {
                touchSessionType.reset();
            }
            else if (!touchSessionType.has_value())
            {
                touchSessionType.emplace(std::min(points.count(), 2));
            }

            // TODO: when a single-touch session becomes multi-touch, trigger a MouseRelease
            if (touchSessionType > points.count())
            {
                // During a multi-touch session, only process multi-touch events
                return true;
            }

            if (points.count() == 2 ||
                points.count() == 1 && MainWidget::Instance()->GetEditMode() == LM::EditMode::Mode_None)
            {
                // check if it's a view adjustment event
                if (event->type() == QEvent::TouchUpdate)
                {
                    if (!touchSession.has_value())
                    {
                        touchSession = TouchController();
                    }
                    touchSession->Update(points, m_camera);
                    update();
                }
                else
                {
                    touchSession.reset();
                }
            }
            else
            {
                touchSession.reset();
            }
            return true;
        }

        if (touchSession.has_value() &&
            (event->type() == QEvent::MouseMove ||
                event->type() == QEvent::MouseButtonPress ||
                event->type() == QEvent::MouseButtonDblClick ||
                event->type() == QEvent::MouseButtonRelease))
        {
            // During touch view adjument, disable editing
            return true;
        }

        return QWidget::event(event);
    }

    QVector3D MapViewGL::PointerDirection(QPoint cursor) const
    {
        auto halfHeight = height() / 2;
        auto halfWidth = width() / 2;
        
        auto focalPlanDistance = halfHeight / std::tan(M_PI / 6); // 60 deg FOV
        auto dirY = halfHeight - cursor.y();
        auto dirX = cursor.x() - halfWidth;
        QVector3D dir(dirX, dirY, -focalPlanDistance);
        auto rtn = m_camera.rotation().rotatedVector(dir);
        rtn.normalize();
        return rtn;
    }

    QPointF MapViewGL::PixelLocation(QVector3D globalDir) const
    {
        QVector3D localPos = m_camera.toMatrix().mapVector(globalDir);

        auto halfHeight = height() / 2;
        auto halfWidth = width() / 2;

        auto focalPlanDistance = halfHeight / std::tan(M_PI / 6); // 60 deg FOV
        auto scale = static_cast<float>(-focalPlanDistance) / localPos.z();
        auto xPixel = scale * localPos.x() + halfWidth;
        auto yPixel = -scale * localPos.y() + halfHeight;
        return QPointF(xPixel, yPixel);
    }

    QVector2D MapViewGL::PointerOnGround(QPoint cursor) const
    {
        QVector3D dir = PointerDirection(cursor);
        auto length = -m_camera.translation().z() / dir.z();
        return QVector2D(m_camera.translation() + length * dir);
    }

    float MapViewGL::Zoom() const
    {
        QVector3D rayOnGround = PointerOnGround(lastMousePos);
        return 100 / rayOnGround.distanceToPoint(m_camera.translation());
    }

    void MapViewGL::SetViewFromReplay(Transform3D t)
    {
        m_camera.setTranslation(t.translation());
        m_camera.setScale(t.scale());
        m_camera.setRotation(t.rotation());
        update();
    }

    void MapViewGL::UpdateRayHit(QPoint screen, bool fromReplay)
    {
        if (fromReplay)
        {
            lastMousePos = screen; // restore Zoom()
        }
        auto currGroundPos = PointerOnGround(screen);
        g_PointerOnGround[0] = currGroundPos.x();
        g_PointerOnGround[1] = currGroundPos.y();

        auto rayDir = PointerDirection(screen);
        auto pointerRayDir = odr::Vec3D{ rayDir.x(), rayDir.y(), rayDir.z() };
        g_CameraPosition = odr::Vec3D{ m_camera.translation().x(), m_camera.translation().y(), m_camera.translation().z() };
        RayCastQuery ray{
            g_CameraPosition,
            pointerRayDir
        };
        auto hitInfo = SpatialIndexer::Instance()->RayCast(ray);

        if (hitInfo.roadID != g_PointerRoadID && !g_PointerRoadID.empty())
        {
            auto prevHL = IDGenerator::ForType(IDType::Road)->GetByID<Road>(g_PointerRoadID);
            if (prevHL != nullptr)
            {
                prevHL->EnableHighlight(false);
                if (prevHL->generated.junction != "-1")
                {
                    (IDGenerator::ForType(IDType::Junction)->GetByID<Junction>(prevHL->generated.junction))->Hide(false);
                }
            }
        }
        g_PointerRoadID.clear();

        if (hitInfo.hit)
        {
            g_PointerRoadID = hitInfo.roadID;
            g_PointerRoadS = hitInfo.s;
            g_PointerLane = hitInfo.lane;
            auto hitRoad = IDGenerator::ForType(IDType::Road)->GetByID<Road>(g_PointerRoadID);
            hitRoad->EnableHighlight(true);
            if (hitRoad->generated.junction != "-1")
            {
                IDGenerator::ForType(IDType::Junction)->GetByID<Junction>(hitRoad->generated.junction)->Hide(true);
            }
        }

        odr::Vec3D pointerOnGround3D{ g_PointerOnGround[0], g_PointerOnGround[1], 0 };

        auto pointerVehicle = LM::SpatialIndexerDynamic::Instance()->RayCast(LM::g_CameraPosition, pointerRayDir);
        if (pointerVehicle != g_PointerVehicle)
        {
            auto prevHighlight = IDGenerator::ForType(IDType::Vehicle)->GetByID<Vehicle>(std::to_string(g_PointerVehicle));
            if (prevHighlight != nullptr)
            {
                prevHighlight->EnableRouteVisual(false, LM::ChangeTracker::Instance()->Map());
            }
        }
        if (pointerVehicle != -1)
        {
            IDGenerator::ForType(IDType::Vehicle)->GetByID<Vehicle>(std::to_string(pointerVehicle))->EnableRouteVisual(true,
                LM::ChangeTracker::Instance()->Map());
        };

        g_PointerVehicle = pointerVehicle;
    }

    QRect MapViewGL::SceneTiedLayover::renderedRect(QMatrix4x4 worldToView) const
    {
        auto homo = worldToView * QVector4D(pos[0], pos[1], pos[2], 1.0);
        int screenX = (homo.x() / homo.w() + 1) / 2 * g_mapViewGL->width();
        int screenY = (1 - homo.y() / homo.w()) / 2 * g_mapViewGL->height();
        auto rect = lwOffset;
        return QRect(screenX - rect.width() / 2 + rect.left(),
            screenY - rect.height() / 2 + rect.top(),
            rect.width(), rect.height());
    }
}
