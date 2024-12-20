#include "map_view_gl.h"
#include <QOpenGLShaderProgram>
#include <QMouseEvent>
#include <QKeyEvent>

#include <spdlog/spdlog.h>
#include "id_generator.h"
#include "spatial_indexer.h"
#include "triangulation.h"

namespace RoadRunner
{
    MapViewGL* g_mapViewGL;
    std::string g_PointerRoadID;
    double g_PointerRoadS;
    int g_PointerLane;
    odr::Vec2D g_PointerOnGround;

    MapViewGL::MapViewGL():
        shader(":/shaders/simple.vert", ":/shaders/simple.frag"),
        m_vbo(QOpenGLBuffer::VertexBuffer) // actually the default, so default constructor would have been enough
    {
        shader.m_uniformNames.append("worldToView");

        // move camera a little back and up
        m_camera.translate(0, -200, 250);
        // look slightly down
        m_camera.rotate(30, m_camera.right());

        // look slightly right
        //m_camera.rotate(-25, QVector3D(0.0f, 1.0f, 0.0f));

        g_mapViewGL = this;

        memset(pressedKeys, 0, sizeof(pressedKeys) / sizeof(bool));
        m_vertexBufferCount = 0;
    }

    unsigned int MapViewGL::AddQuads(const odr::Line3D& lBorder, const odr::Line3D& rBorder, QColor color)
    {
        assert(lBorder.size() == rBorder.size());

        m_vao.bind();
        m_vbo.bind();

        unsigned int gid = std::stoi(IDGenerator::ForGraphics()->GenerateID(this));
        std::set<GLuint> vids;
        const auto vertexBufferChangeBegin = m_vertexBufferCount;

        for (int i = 0; i < lBorder.size() - 1; ++i)
        {
            auto l0 = lBorder[i], l1 = lBorder[i + 1];
            auto r0 = rBorder[i], r1 = rBorder[i + 1];

            auto v11 = m_vertexBufferCount++;
            m_vertexBufferData[v11] = Vertex(QVector3D(l0[0], l0[1], l0[2]), color, gid);
            vids.emplace(v11);
            auto v12 = m_vertexBufferCount++;
            m_vertexBufferData[v12] = Vertex(QVector3D(r0[0], r0[1], r0[2]), color, gid);
            vids.emplace(v12);
            auto v13 = m_vertexBufferCount++;
            m_vertexBufferData[v13] = Vertex(QVector3D(l1[0], l1[1], l1[2]), color, gid);
            vids.emplace(v13);

            auto v21 = m_vertexBufferCount++;
            m_vertexBufferData[v21] = Vertex(QVector3D(l1[0], l1[1], l1[2]), color, gid);
            vids.emplace(v21);
            auto v22 = m_vertexBufferCount++;
            m_vertexBufferData[v22] = Vertex(QVector3D(r0[0], r0[1], r0[2]), color, gid);
            vids.emplace(v22);
            auto v23 = m_vertexBufferCount++;
            m_vertexBufferData[v23] = Vertex(QVector3D(r1[0], r1[1], r1[2]), color, gid);
            vids.emplace(v23);
        }
        auto ptr_v = m_vbo.mapRange(vertexBufferChangeBegin * sizeof(Vertex), 
            (m_vertexBufferCount - vertexBufferChangeBegin) * sizeof(Vertex), 
            QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
        memcpy(ptr_v, m_vertexBufferData.data() + vertexBufferChangeBegin,
            (m_vertexBufferCount - vertexBufferChangeBegin) * sizeof(Vertex));
        m_vbo.unmap();
        
        m_vbo.release();
        m_vao.release();

        idToVids.emplace(gid, vids);
        return gid;
    }

    unsigned int MapViewGL::AddPoly(const odr::Line3D& boundary, QColor color)
    {
        m_vao.bind();
        m_vbo.bind();

        auto gid = std::stoi(IDGenerator::ForGraphics()->GenerateID(this));
        std::set<GLuint> vids;
        const auto vertexBufferChangeBegin = m_vertexBufferCount;

        for (auto tri : Triangulate_2_5d(boundary))
        {
            auto p1 = boundary[std::get<0>(tri)];
            auto v1 = m_vertexBufferCount++;
            vids.emplace(v1);
            m_vertexBufferData[v1] = Vertex(QVector3D(p1[0], p1[1], p1[2]), color, gid);

            auto p2 = boundary[std::get<1>(tri)];
            auto v2 = m_vertexBufferCount++;
            vids.emplace(v2);
            m_vertexBufferData[v2] = Vertex(QVector3D(p2[0], p2[1], p2[2]), color, gid);

            auto p3 = boundary[std::get<2>(tri)];
            auto v3 = m_vertexBufferCount++;
            vids.emplace(v3);
            m_vertexBufferData[v3] = Vertex(QVector3D(p3[0], p3[1], p3[2]), color, gid);
        }

        auto ptr_v = m_vbo.mapRange(vertexBufferChangeBegin * sizeof(Vertex),
            (m_vertexBufferCount - vertexBufferChangeBegin) * sizeof(Vertex),
            QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
        memcpy(ptr_v, m_vertexBufferData.data() + vertexBufferChangeBegin,
            (m_vertexBufferCount - vertexBufferChangeBegin) * sizeof(Vertex));
        m_vbo.unmap();

        m_vbo.release();
        m_vao.release();

        idToVids.emplace(gid, vids);
        return gid;
    }

    void MapViewGL::UpdateItem(unsigned int id, QColor color)
    {
        m_vao.bind();
        m_vbo.bind();
        for (auto vid : idToVids.at(id))
        {
            m_vertexBufferData[vid].r = color.redF();
            m_vertexBufferData[vid].g = color.greenF();
            m_vertexBufferData[vid].b = color.blueF();

            auto ptr_v = m_vbo.mapRange(vid * sizeof(Vertex), sizeof(Vertex),
                QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
            assert(ptr_v != nullptr);
            memcpy(ptr_v, m_vertexBufferData.data() + vid, sizeof(Vertex));
            m_vbo.unmap();
        }

        m_vao.release();
        m_vbo.release();
    }

    void MapViewGL::RemoveItem(unsigned int id)
    {
        if (!Road::ClearingMap)
        {
            m_vao.bind();
            m_vbo.bind();
        }

        IDGenerator::ForGraphics()->FreeID(std::to_string(id));

        const auto& vidsToRemove = idToVids.at(id);
        for (auto vid_it = vidsToRemove.rbegin(); vid_it != vidsToRemove.rend(); ++vid_it)
        {
            auto vid = *vid_it;
            if (!Road::ClearingMap && m_vertexBufferCount != 0 && m_vertexBufferCount - 1 != vid)
            {
                assert(m_vertexBufferData[vid].objectID == id);
                auto objectToMove = m_vertexBufferData[m_vertexBufferCount - 1].objectID;
                auto oldVIt = idToVids.at(objectToMove).find(m_vertexBufferCount - 1);
                idToVids.at(objectToMove).erase(oldVIt);

                assert(idToVids.at(objectToMove).find(vid) == idToVids.at(objectToMove).end());
                idToVids.at(objectToMove).emplace(vid);
                m_vertexBufferData[vid] = m_vertexBufferData[m_vertexBufferCount - 1];
                auto ptr_v = m_vbo.mapRange(vid * sizeof(Vertex), sizeof(Vertex), 
                    QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
                assert(ptr_v != nullptr);
                memcpy(ptr_v, m_vertexBufferData.data() + vid, sizeof(Vertex));
                m_vbo.unmap();
            }
            m_vertexBufferCount--;
        }
        idToVids.erase(id);

        if (!Road::ClearingMap)
        {
            m_vao.release();
            m_vbo.release();
        }
    }

    int MapViewGL::VBufferUseage_pct() const
    {
        return static_cast<float>(m_vertexBufferCount) / m_vertexBufferData.size() * 100.0;
    }

    void MapViewGL::initializeGL()
    {
        shader.create();

        // tell OpenGL to show only faces whose normal vector points towards us
        glDisable(GL_CULL_FACE);
        // enable depth testing, important for the grid and for the drawing order of several objects
        glEnable(GL_DEPTH_TEST);

        m_vao.create();
        m_vao.bind();

        m_vbo.create();
        m_vbo.bind();
        m_vbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
        int vertexMemSize = m_vertexBufferData.size() * sizeof(Vertex);
        m_vbo.allocate(m_vertexBufferData.data(), vertexMemSize);

        auto shaderProgramm = shader.shaderProgram();
        shaderProgramm->enableAttributeArray(0);
        shaderProgramm->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(Vertex));
        shaderProgramm->enableAttributeArray(1); // array with index/id 1
        shaderProgramm->setAttributeBuffer(1, GL_FLOAT, offsetof(Vertex, r), 3, sizeof(Vertex));

        // Release (unbind) all
        m_vao.release();
        m_vbo.release();
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
        auto shaderProgramm = shader.shaderProgram();
        shaderProgramm->bind();
        shaderProgramm->setUniformValue(shader.m_uniformIDs[0], m_worldToView);
        m_vao.bind();
        glDrawArrays(GL_TRIANGLES, 0, m_vertexBufferCount);
        m_vao.release();
        shaderProgramm->release();
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
            emit(MousePerformedAction(event));
        }
    }

    void MapViewGL::mouseDoubleClickEvent(QMouseEvent* evt)
    {
        emit(MousePerformedAction(evt));
    }

    void MapViewGL::mouseMoveEvent(QMouseEvent* event)
    {
        auto currGroundPos = PointerOnGround(event->pos());
        g_PointerOnGround[0] = currGroundPos.x();
        g_PointerOnGround[1] = currGroundPos.y();
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
            m_camera.translate(lastGroundPos - currGroundPos);
        }
        else
        {
            changeViewPoint = false;
        }

        auto rayDir = PointerDirection(event->pos());
        RayCastQuery ray{
            odr::Vec3D{m_camera.translation().x(), m_camera.translation().y(), m_camera.translation().z()},
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

        if (!changeViewPoint)
        {
            emit(MousePerformedAction(event));
        }
        renderLater();
        lastMousePos = event->pos();
    }

    void MapViewGL::mouseReleaseEvent(QMouseEvent* event)
    {
        dragRotFixedRay.reset();
        dragPan = false;
        if (event->button() == Qt::LeftButton)
        {
            emit(MousePerformedAction(event));
        }
    }

    void MapViewGL::wheelEvent(QWheelEvent* event)
    {
        auto dir = event->angleDelta().y() > 0 ? 1 : -1;
        auto delta = dir * PointerDirection(lastMousePos) * 10;
        m_camera.translate(delta);
        renderLater();
    }

    void MapViewGL::keyPressEvent(QKeyEvent* event)
    {
        auto flatForward = m_camera.forward().toVector2D();
        flatForward.normalize();
        flatForward *= 5;
        if (event->key() == Qt::Key_W)
        {
            pressedKeys[Qt::Key_W] = true;
        }
        if (event->key() == Qt::Key_S)
        {
            pressedKeys[Qt::Key_S] = true;
        }
        if (event->key() == Qt::Key_A)
        {
            pressedKeys[Qt::Key_A] = true;
        }
        if (event->key() == Qt::Key_D)
        {
            pressedKeys[Qt::Key_D] = true;
        }
        if (event->key() == Qt::Key_Q)
        {
            pressedKeys[Qt::Key_Q] = true;
        }
        if (event->key() == Qt::Key_E)
        {
            pressedKeys[Qt::Key_E] = true;
        }

        if (pressedKeys[Qt::Key_W])
        {
            m_camera.translate(flatForward.x(), flatForward.y(), 0);
        }
        if (pressedKeys[Qt::Key_S])
        {
            m_camera.translate(-flatForward.x(), -flatForward.y(), 0);
        }
        if (pressedKeys[Qt::Key_A])
        {
            m_camera.translate(-flatForward.y(), flatForward.x(), 0);
        }
        if (pressedKeys[Qt::Key_D])
        {
            m_camera.translate(flatForward.y(), -flatForward.x(), 0);
        }
        if (pressedKeys[Qt::Key_Q])
        {
            m_camera.rotate(-5, QVector3D(0, 0, 1));
        }
        if (pressedKeys[Qt::Key_E])
        {
            m_camera.rotate(5, QVector3D(0, 0, 1));
        }

        emit(KeyPerformedAction(event));
        // update cached world2view matrix
        renderLater();
    }

    void MapViewGL::keyReleaseEvent(QKeyEvent* event)
    {
        if (event->key() == Qt::Key_W)
        {
            pressedKeys[Qt::Key_W] = false;
        }
        if (event->key() == Qt::Key_S)
        {
            pressedKeys[Qt::Key_S] = false;
        }
        if (event->key() == Qt::Key_A)
        {
            pressedKeys[Qt::Key_A] = false;
        }
        if (event->key() == Qt::Key_D)
        {
            pressedKeys[Qt::Key_D] = false;
        }
        if (event->key() == Qt::Key_Q)
        {
            pressedKeys[Qt::Key_Q] = false;
        }
        if (event->key() == Qt::Key_E)
        {
            pressedKeys[Qt::Key_E] = false;
        }
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
}
