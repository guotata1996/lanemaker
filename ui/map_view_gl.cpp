#include "map_view_gl.h"
#include <QOpenGLShaderProgram>
#include <QMouseEvent>
#include <QKeyEvent>

#include <spdlog/spdlog.h>
#include "id_generator.h"
#include "spatial_indexer.h"
#include "triangulation.h"
#include "action_manager.h"
#include "constants.h"

namespace RoadRunner
{
    MapViewGL* g_mapViewGL;
    std::string g_PointerRoadID;
    double g_PointerRoadS;
    int g_PointerLane;
    odr::Vec2D g_PointerOnGround;
    odr::Vec3D g_CameraPosition;

    MapViewGL::MapViewGL() :
        permanentBuffer(1 << 24), temporaryBuffer(1 << 18)
    {
        g_mapViewGL = this;

        ResetCamera();
    }

    void MapViewGL::ResetCamera()
    {
        m_camera.setTranslation(0, -200, 250);
        m_camera.setRotation(30, QVector3D(1, 0, 0));
    }

    unsigned int MapViewGL::AddQuads(const odr::Line3D& lBorder, const odr::Line3D& rBorder, QColor color, bool temporary)
    {
        unsigned int gid = std::stoi(IDGenerator::ForGraphics(temporary)->GenerateID(this));
        if (temporary)
        {
            temporaryBuffer.AddQuads(gid, lBorder, rBorder, color); // TODO: check success
        }
        else
        {
            permanentBuffer.AddQuads(gid, lBorder, rBorder, color); // TODO: check success
        }
        return gid;
    }

    unsigned int MapViewGL::AddPoly(const odr::Line3D& boundary, QColor color, bool temporary)
    {
        auto gid = std::stoi(IDGenerator::ForGraphics(temporary)->GenerateID(this));
        if (temporary)
        {
            temporaryBuffer.AddPoly(gid, boundary, color);
        }
        else
        {
            permanentBuffer.AddPoly(gid, boundary, color);
        }
        return gid;
    }

    void MapViewGL::UpdateItem(unsigned int id, QColor color, bool temporary)
    {
        if (!temporary)
        {
            permanentBuffer.UpdateItem(id, color);
        }
        else
        {
            temporaryBuffer.UpdateItem(id, color);
        }
    }

    void MapViewGL::RemoveItem(unsigned int id, bool temporary)
    {
        if (!temporary)
        {
            permanentBuffer.RemoveItem(id);
        }
        else
        {
            temporaryBuffer.RemoveItem(id);
        }
        IDGenerator::ForGraphics(temporary)->FreeID(std::to_string(id));
    }

    int MapViewGL::VBufferUseage_pct() const
    {
        return permanentBuffer.Useage_pct();
    }

    void MapViewGL::initializeGL()
    {
        // tell OpenGL to show only faces whose normal vector points towards us
        glDisable(GL_CULL_FACE);
        // enable depth testing, important for the grid and for the drawing order of several objects
        glEnable(GL_DEPTH_TEST);

        permanentBuffer.Initialize();
        temporaryBuffer.Initialize();
    }

    void MapViewGL::resizeGL(int width, int height)
    {
        m_projection.setToIdentity();
        // create projection matrix, i.e. camera lens
        m_projection.perspective(
            /* vertical angle */ 60.0f,
            /* aspect ratio */   width / float(height),
            /* near */           0.1f,
            /* far */            1000.0f
        );
        // Mind: to not use 0.0 for near plane, otherwise depth buffering and depth testing won't work!
    }

    void MapViewGL::paintGL()
    {
        // update cached world2view matrix
        m_worldToView = m_projection * m_camera.toMatrix() * m_transform.toMatrix();

        const qreal retinaScale = devicePixelRatio(); // needed for Macs with retina display
        glViewport(0, 0, width() * retinaScale, height() * retinaScale);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.1f, 0.15f, 0.3f, 1.0f);

        for (auto buffer : {&permanentBuffer, &temporaryBuffer})
        {
            auto nVertex = buffer->Bind(m_worldToView);
            glDrawArrays(GL_TRIANGLES, 0, nVertex);
            buffer->Unbind();
        }
    }

    void MapViewGL::mousePressEvent(QMouseEvent* event)
    {
        bool ctrlPressed = event->modifiers() & Qt::CTRL;
        if (event->button() == Qt::RightButton && !ctrlPressed)
        {
            lastMousePos = event->pos();
            dragRotFixedRay = PointerDirection(lastMousePos);
        }
        else if (event->button() == Qt::MiddleButton)
        {
            lastMousePos = event->pos();
            dragPan = true;
        }
        else
        {
            RoadRunner::ActionManager::Instance()->Record(event);
            emit(MousePerformedAction(event));
        }
    }

    void MapViewGL::mouseDoubleClickEvent(QMouseEvent* evt)
    {
        if (evt->button() == Qt::LeftButton)
        {
            RoadRunner::ActionManager::Instance()->Record(evt);
            emit(MousePerformedAction(evt));
        }
    }

    void MapViewGL::mouseMoveEvent(QMouseEvent* event)
    {
        UpdateRayHit(event->pos());
        
        bool changeViewPoint = true;
        if (dragRotFixedRay.has_value())
        {
            if (event->x() < 0 || event->x() > width() ||
                event->y() < 0 || event->y() > height())
            {
                return;
            }
            
            int i;
            const int MaxIter = 500;
            const auto backupRotation = m_camera.rotation();
            for (i = 0; i != MaxIter; ++i)
            {
                double step = i < MaxIter / 2 ? 1 : 0.1;
                int tol = i < MaxIter / 2 ? 1 : 3;
                QPointF rayPixel = PixelLocation(dragRotFixedRay.value());

                auto xError = std::abs(event->x() - rayPixel.x());
                auto yError = std::abs(event->y() - rayPixel.y());

                if (xError <= tol && yError <= tol)
                {
                    break;
                }
                if (xError > yError)
                {
                    if (event->x() > rayPixel.x() + tol)
                    {
                        m_camera.rotate(step, QVector3D(0, 0, 1));
                    }
                    else
                    {
                        m_camera.rotate(-step, QVector3D(0, 0, 1));
                    }
                }
                else
                {
                    if (event->y() > rayPixel.y() + tol)
                    {
                        m_camera.rotate(step, m_camera.right());
                    }
                    else
                    {
                        m_camera.rotate(-step, m_camera.right());
                    }
                }                
            }

            if (i == MaxIter)
            {
                spdlog::trace("Camera Angle Adj reach max iteration");
                m_camera.setRotation(backupRotation);
            }
            else if (!m_camera.isRotationValid())
            {
                m_camera.setRotation(backupRotation);
            }
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
            RoadRunner::ActionManager::Instance()->Record(event);
            emit(MousePerformedAction(event));
        }
        renderLater();
        lastMousePos = event->pos();
    }

    void MapViewGL::mouseReleaseEvent(QMouseEvent* event)
    {
        if (dragRotFixedRay.has_value() || dragPan)
        {
            dragRotFixedRay.reset();
            dragPan = false;
        }
        else
        {
            RoadRunner::ActionManager::Instance()->Record(event);
            emit(MousePerformedAction(event));
        }
    }

    void MapViewGL::wheelEvent(QWheelEvent* event)
    {
        auto dir = event->angleDelta().y() > 0 ? 1 : -1;
        auto delta = dir * PointerDirection(lastMousePos) * 10;
        m_camera.translate(delta);
        ActionManager::Instance()->Record(m_camera);

        renderLater();
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
            RoadRunner::ActionManager::Instance()->Record(m_camera);
        }
        else
        {
            RoadRunner::ActionManager::Instance()->Record(event);
            emit(KeyPerformedAction(event));
        }
        // update cached world2view matrix
        renderLater();
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
        renderLater();
    }

    void MapViewGL::UpdateRayHit(QPoint screen)
    {
        // TODO: g_PointerVehicle.lock()->EnableRouteVisual(false, RoadRunner::ChangeTracker::Instance()->Map());
        auto currGroundPos = PointerOnGround(screen);
        g_PointerOnGround[0] = currGroundPos.x();
        g_PointerOnGround[1] = currGroundPos.y();

        auto rayDir = PointerDirection(screen);
        g_CameraPosition = odr::Vec3D{ m_camera.translation().x(), m_camera.translation().y(), m_camera.translation().z() };
        RayCastQuery ray{
            g_CameraPosition,
            odr::Vec3D{rayDir.x(), rayDir.y(), rayDir.z()}
        };
        auto hitInfo = SpatialIndexer::Instance()->RayCast(ray);

        if (hitInfo.roadID != g_PointerRoadID && !g_PointerRoadID.empty())
        {
            auto prevHL = static_cast<Road*>(IDGenerator::ForRoad()->GetByID(g_PointerRoadID));
            if (prevHL != nullptr)
            {
                prevHL->EnableHighlight(false);

            }
        }
        g_PointerRoadID.clear();

        if (hitInfo.hit)
        {
            g_PointerRoadID = hitInfo.roadID;
            g_PointerRoadS = hitInfo.s;
            g_PointerLane = hitInfo.lane;
            static_cast<Road*>(IDGenerator::ForRoad()->GetByID(g_PointerRoadID))->EnableHighlight(true);
        }
    }
}
