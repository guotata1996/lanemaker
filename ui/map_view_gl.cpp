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

    MapViewGL::MapViewGL():
        shader(":/shaders/simple.vert", ":/shaders/simple.frag"),
        m_vbo(QOpenGLBuffer::VertexBuffer), // actually the default, so default constructor would have been enough
        m_ebo(QOpenGLBuffer::IndexBuffer) // make this an Index Buffer
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
        m_vertexBufferCount = m_elementCount = 0;
    }

    unsigned int MapViewGL::AddQuads(const odr::Line3D& lBorder, const odr::Line3D& rBorder, QColor color)
    {
        assert(lBorder.size() == rBorder.size());

        m_vao.bind();
        m_vbo.bind();

        std::vector<GLuint> vids;
        const size_t vertexBufferChangeBegin = m_vertexBufferCount;

        for (int i = 0; i != lBorder.size(); ++i)
        {
            auto l0 = lBorder[i];
            auto id_l = m_vertexBufferCount++;
            m_vertexBufferData[id_l] = Vertex(QVector3D(l0[0], l0[1], l0[2]), color);
            vids.push_back(id_l);

            auto r0 = rBorder[i];
            auto id_r = m_vertexBufferCount++;
            m_vertexBufferData[id_r] = Vertex(QVector3D(r0[0], r0[1], r0[2]), color);
            vids.push_back(id_r);
        }
        auto ptr_v = m_vbo.mapRange(vertexBufferChangeBegin * sizeof(Vertex), 
            (m_vertexBufferCount - vertexBufferChangeBegin) * sizeof(Vertex), 
            QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
        memcpy(ptr_v, m_vertexBufferData.data() + vertexBufferChangeBegin,
            (m_vertexBufferCount - vertexBufferChangeBegin) * sizeof(Vertex));
        m_vbo.unmap();

        std::vector<GLuint> eids;
        const size_t elementChangeBegin = m_elementCount;
        for (int i = 0; i < lBorder.size() - 1; ++i)
        {
            auto ea = m_elementCount++;
            eids.push_back(ea);
            m_elementBufferData[3 * ea + 0] = vids[2 * i + 0];
            m_elementBufferData[3 * ea + 1] = vids[2 * i + 1];
            m_elementBufferData[3 * ea + 2] = vids[2 * i + 2];

            auto eb = m_elementCount++;
            eids.push_back(eb);
            m_elementBufferData[3 * eb + 0] = vids[2 * i + 1];
            m_elementBufferData[3 * eb + 1] = vids[2 * i + 2];
            m_elementBufferData[3 * eb + 2] = vids[2 * i + 3];
        }
        auto ptr_e = m_ebo.mapRange(3 * elementChangeBegin * sizeof(GLuint), 
            3 * (m_elementCount - elementChangeBegin) * sizeof(GLuint), 
            QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
        memcpy(ptr_e, m_elementBufferData.data() + 3 * elementChangeBegin,
            3 * (m_elementCount - elementChangeBegin) * sizeof(GLuint));
        m_ebo.unmap();
        
        m_vbo.release();
        m_vao.release();

        auto gid = std::stoi(IDGenerator::ForGraphics()->GenerateID(this));
        idToEids.emplace(gid, eids);
        idToVids.emplace(gid, vids);
        return gid;
    }

    unsigned int MapViewGL::AddPoly(const odr::Line3D& boundary, QColor color)
    {
        m_vao.bind();
        m_vbo.bind();

        std::vector<GLuint> vids;
        const size_t vertexBufferChangeBegin = m_vertexBufferCount;
        for (const auto& p : boundary)
        {
            auto vid = m_vertexBufferCount++;
            vids.push_back(vid);
            m_vertexBufferData[vid] = Vertex(QVector3D(p[0], p[1], p[2]), color);
        }
        auto ptr_v = m_vbo.mapRange(vertexBufferChangeBegin * sizeof(Vertex),
            (m_vertexBufferCount - vertexBufferChangeBegin) * sizeof(Vertex),
            QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
        memcpy(ptr_v, m_vertexBufferData.data() + vertexBufferChangeBegin,
            (m_vertexBufferCount - vertexBufferChangeBegin) * sizeof(Vertex));
        m_vbo.unmap();

        std::vector<GLuint> eids;
        const size_t elementChangeBegin = m_elementCount;
        for (auto tri : Triangulate_2_5d(boundary))
        {
            auto eid = m_elementCount++;
            eids.push_back(eid);
            m_elementBufferData[3 * eid] = std::get<0>(tri) + vertexBufferChangeBegin;
            m_elementBufferData[3 * eid + 1] = std::get<1>(tri) + vertexBufferChangeBegin;
            m_elementBufferData[3 * eid + 2] = std::get<2>(tri) + vertexBufferChangeBegin;
        }
        auto ptr_e = m_ebo.mapRange(3 * elementChangeBegin * sizeof(GLuint),
            3 * (m_elementCount - elementChangeBegin) * sizeof(GLuint),
            QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
        memcpy(ptr_e, m_elementBufferData.data() + 3 * elementChangeBegin,
            3 * (m_elementCount - elementChangeBegin) * sizeof(GLuint));
        m_ebo.unmap();

        m_vbo.release();
        m_vao.release();

        auto gid = std::stoi(IDGenerator::ForGraphics()->GenerateID(this));
        idToEids.emplace(gid, eids);
        idToVids.emplace(gid, vids);
        return gid;
    }

    void MapViewGL::RemoveItem(unsigned int id)
    {
        if (!Road::ClearingMap)
        {
            m_vao.bind();
            m_vbo.bind();
        }

        IDGenerator::ForGraphics()->FreeID(std::to_string(id));

        for (auto eid : idToEids.at(id))
        {
            if (!Road::ClearingMap && m_elementCount != 0 && m_elementCount - 1 != eid)
            {
                // swap eid and m_elementCount - 1
                m_elementBufferData[3 * eid]     = m_elementBufferData[3 * (m_elementCount - 1)];
                m_elementBufferData[3 * eid + 1] = m_elementBufferData[3 * (m_elementCount - 1) + 1];
                m_elementBufferData[3 * eid + 2] = m_elementBufferData[3 * (m_elementCount - 1) + 2];

                auto ptr_e = m_ebo.mapRange(3 * eid * sizeof(GLuint), 3 * sizeof(GLuint),
                    QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
                memcpy(ptr_e, m_elementBufferData.data() + 3 * eid, 3 * sizeof(GLuint));
                m_ebo.unmap();
            }
            m_elementCount--;
        }
        idToEids.erase(id);

        for (auto vid : idToVids.at(id))
        {
            if (!Road::ClearingMap && m_vertexBufferCount != 0 && m_vertexBufferCount - 1 != vid)
            {
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

    void MapViewGL::initializeGL()
    {
        shader.create();

        std::fill(m_elementBufferData.begin(), m_elementBufferData.end(), 0);

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

        m_ebo.create();
        m_ebo.bind();
        m_ebo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
        int elementMemSize = m_elementBufferData.size() * sizeof(GLuint);
        m_ebo.allocate(m_elementBufferData.data(), elementMemSize);

        auto shaderProgramm = shader.shaderProgram();
        shaderProgramm->enableAttributeArray(0);
        shaderProgramm->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(Vertex));
        shaderProgramm->enableAttributeArray(1); // array with index/id 1
        shaderProgramm->setAttributeBuffer(1, GL_FLOAT, offsetof(Vertex, r), 3, sizeof(Vertex));

        // Release (unbind) all
        m_vao.release();
        m_vbo.release();
        m_ebo.release();
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
        glDrawElements(GL_TRIANGLES, m_elementCount * 3, GL_UNSIGNED_INT, nullptr);
        m_vao.release();
        shaderProgramm->release();
    }

    void MapViewGL::mousePressEvent(QMouseEvent* event)
    {
        if (event->button() == Qt::RightButton)
        {
            lastMousePos = event->pos();
            dragRotFixedRay = PointerDirection(lastMousePos);
        }
        else if (event->button() == Qt::MiddleButton)
        {
            lastMousePos = event->pos();
            dragPan = true;
        }
    }

    void MapViewGL::mouseMoveEvent(QMouseEvent* event)
    {
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
            else
            {
                renderLater();
            }
        }
        else if (dragPan)
        {
            auto lastGroundPos = PointerOnGround(lastMousePos);
            auto currGroundPos = PointerOnGround(event->pos());
            m_camera.translate(lastGroundPos - currGroundPos);
            renderLater();
        }
        else
        {
            auto rayDir = PointerDirection(event->pos());
            OpenGLWindow::mouseMoveEvent(event);
            RayCastQuery ray{
                odr::Vec3D{m_camera.translation().x(), m_camera.translation().y(), m_camera.translation().z()},
                odr::Vec3D{rayDir.x(), rayDir.y(), rayDir.z()}
            };
            auto hitInfo = SpatialIndexer::Instance()->RayCast(ray);
            if (hitInfo.hit)
            {
                spdlog::info("Hit: Road {} @ {}, {}, {}", hitInfo.roadID, hitInfo.hitPos[0], hitInfo.hitPos[1], hitInfo.hitPos[2]);
            }
        }
        lastMousePos = event->pos();
    }

    void MapViewGL::mouseReleaseEvent(QMouseEvent* event)
    {
        dragRotFixedRay.reset();
        dragPan = false;
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
}
