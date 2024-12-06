#include "map_view_gl.h"
#include <QOpenGLShaderProgram>

#include "id_generator.h"
#include <QMouseEvent>
#include <QKeyEvent>

#include <spdlog/spdlog.h>
#include "spatial_indexer.h"

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
    }

    unsigned int MapViewGL::AddQuads(const odr::Line3D& lBorder, const odr::Line3D& rBorder)
    {
        assert(lBorder.size() == rBorder.size());

        m_vao.bind();
        m_vbo.bind();

        std::vector<GLuint> vids;
        for (int i = 0; i != lBorder.size(); ++i)
        {
            auto l0 = lBorder[i];
            auto id_ls = IDGenerator::ForVertex()->GenerateID(this);
            auto id_l = std::stoi(id_ls);
            m_vertexBufferData[id_l] = Vertex(QVector3D(l0[0], l0[1], l0[2]), QColor(255, 128, 64));
            vids.push_back(id_l);

            auto ptr_l = m_vbo.mapRange(id_l * sizeof(Vertex), sizeof(Vertex), QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
            memcpy(ptr_l, m_vertexBufferData.data() + id_l, sizeof(Vertex));
            m_vbo.unmap();

            auto r0 = rBorder[i];
            auto id_rs = IDGenerator::ForVertex()->GenerateID(this);
            auto id_r = std::stoi(id_rs);
            m_vertexBufferData[id_r] = Vertex(QVector3D(r0[0], r0[1], r0[2]), QColor(255, 128, 64));
            vids.push_back(id_r);

            auto ptr_r = m_vbo.mapRange(id_r * sizeof(Vertex), sizeof(Vertex), QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
            memcpy(ptr_r, m_vertexBufferData.data() + id_r, sizeof(Vertex));
            m_vbo.unmap();
        }

        std::vector<GLuint> eids;
        for (int i = 0; i < lBorder.size() - 1; ++i)
        {
            auto ea = std::stoi(IDGenerator::ForElement()->GenerateID(this));
            eids.push_back(ea);
            m_elementBufferData[3 * ea + 0] = vids[2 * i + 0];
            m_elementBufferData[3 * ea + 1] = vids[2 * i + 1];
            m_elementBufferData[3 * ea + 2] = vids[2 * i + 2];

            auto ptr_a = m_ebo.mapRange(3 * ea * sizeof(GLuint), 3 * sizeof(GLuint), QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
            memcpy(ptr_a, m_elementBufferData.data() + 3 * ea, 3 * sizeof(GLuint));
            m_ebo.unmap();

            auto eb = std::stoi(IDGenerator::ForElement()->GenerateID(this));
            eids.push_back(eb);
            m_elementBufferData[3 * eb + 0] = vids[2 * i + 1];
            m_elementBufferData[3 * eb + 1] = vids[2 * i + 2];
            m_elementBufferData[3 * eb + 2] = vids[2 * i + 3];

            auto ptr_b = m_ebo.mapRange(3 * eb * sizeof(GLuint), 3 * sizeof(GLuint), QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
            memcpy(ptr_b, m_elementBufferData.data() + 3 * eb, 3 * sizeof(GLuint));
            m_ebo.unmap();
        }
        
        m_vbo.release();
        //m_ebo.release();
        m_vao.release();

        auto gid = std::stoi(IDGenerator::ForGraphics()->GenerateID(this));
        idToEids.emplace(gid, eids);
        idToVids.emplace(gid, vids);
        return gid;
    }

    void MapViewGL::RemoveItem(unsigned int id)
    {
        m_vao.bind();

        IDGenerator::ForGraphics()->FreeID(std::to_string(id));
        for (auto eid : idToEids.at(id))
        {
            IDGenerator::ForElement()->FreeID(std::to_string(eid));
            m_elementBufferData[3 * eid] = 0;
            m_elementBufferData[3 * eid + 1] = 0;
            m_elementBufferData[3 * eid + 2] = 0;

            auto ptr_a = m_ebo.mapRange(3 * eid * sizeof(GLuint), 3 * sizeof(GLuint), QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
            if (ptr_a != nullptr)
            {
                memcpy(ptr_a, m_elementBufferData.data() + 3 * eid, 3 * sizeof(GLuint));
                m_ebo.unmap();
            }
        }
        idToEids.erase(id);

        for (auto vid : idToVids.at(id))
        {
            IDGenerator::ForVertex()->FreeID(std::to_string(vid));
        }
        idToVids.erase(id);

        m_vao.release();
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
        glDrawElements(GL_TRIANGLES, m_elementBufferData.size(), GL_UNSIGNED_INT, nullptr);
        m_vao.release();
        shaderProgramm->release();
    }

    void MapViewGL::mousePressEvent(QMouseEvent* event)
    {
        if (event->button() == Qt::RightButton)
        {
            dragStartPos = event->pos();
            dragStartRot = m_camera.rotation();
            dragStartRay = PointerDirection(event->pos());
        }
    }

    void MapViewGL::mouseMoveEvent(QMouseEvent* event)
    {
        if (dragStartPos.has_value())
        {
            if (event->x() < 0 || event->x() > width() ||
                event->y() < 0 || event->y() > height())
            {
                return;
            }
            auto halfHeight = height() / 2;
            auto halfWidth = width() / 2;
            auto focalPlanDistance = halfHeight / std::tan(M_PI / 6); // 60 deg FOV

            double startHAngle = std::atan2(dragStartPos->x() - halfWidth, focalPlanDistance);
            double currHAngle = std::atan2(event->x() - halfWidth, focalPlanDistance);
            double deltaHAngle = 180 / M_PI * (currHAngle - startHAngle);
            auto hRot = QQuaternion::fromAxisAndAngle(m_camera.up(), deltaHAngle);
            m_camera.setRotation(hRot);
            
            double startVAngle = std::atan2(dragStartPos->y() - halfHeight, focalPlanDistance);
            double currVAngle = std::atan2(event->y() - halfHeight, focalPlanDistance);
            double deltaVAngle = 180 / M_PI * (currVAngle - startVAngle);
            auto vRot = QQuaternion::fromAxisAndAngle(m_camera.right(), deltaVAngle);
            m_camera.setRotation(dragStartRot.value() * hRot * vRot);

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
                spdlog::info("Hit: {}, {}, {}", hitInfo.hitPos[0], hitInfo.hitPos[1], hitInfo.hitPos[2]);
            }
        }
    }

    void MapViewGL::mouseReleaseEvent(QMouseEvent* event)
    {
        dragStartPos.reset();
    }

    void MapViewGL::keyPressEvent(QKeyEvent* event)
    {
        auto flatForward = m_camera.forward().toVector2D();
        flatForward.normalize();
        if (event->key() == Qt::Key_W)
        {
            m_camera.translate(flatForward.x(), flatForward.y(), 0);
        }
        else if (event->key() == Qt::Key_S)
        {
            m_camera.translate(-flatForward.x(), -flatForward.y(), 0);
        }
        else if (event->key() == Qt::Key_A)
        {
            m_camera.translate(-flatForward.y(), flatForward.x(), 0);
        }
        else if (event->key() == Qt::Key_D)
        {
            m_camera.translate(flatForward.y(), -flatForward.x(), 0);
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
}
