#include "gl_buffer_manage.h"
#include "triangulation.h"
#include "road.h"

#include <QOpenGLShaderProgram>

namespace LM
{
    GLBufferManage::GLBufferManage(unsigned int capacity):
        m_vertexBufferData(capacity),
        m_vbo(QOpenGLBuffer::VertexBuffer),
        shader(":/shaders/simple.vert", ":/shaders/simple.frag")
    {
        shader.m_uniformNames.append("worldToView");
        shader.m_uniformNames.append("objectInfo");
        m_vertexBufferCount = 0;
    }

    void GLBufferManage::Initialize()
    {
        initializeOpenGLFunctions();

        shader.create();

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
        shaderProgramm->enableAttributeArray(2);
        shaderProgramm->setAttributeBuffer(2, GL_FLOAT, offsetof(Vertex, objectID), 1, sizeof(Vertex));

        m_objectInfo = std::make_unique<QOpenGLTexture>(QOpenGLTexture::Target1D);
        m_objectInfo->setData(QImage(MaxObjectID, 1, QImage::Format_Grayscale8));
        Reset();

        shaderProgramm->setUniformValue(shader.m_uniformIDs[1], 0);

        // Release (unbind) all
        m_vao.release();
        m_vbo.release();
    }

    void GLBufferManage::Reset()
    {
        memset(m_objectFlag, static_cast<uint8_t>(ObjectDisplayFlag::Normal), MaxObjectID);
        m_objectInfo->setData(QOpenGLTexture::PixelFormat::Red, QOpenGLTexture::PixelType::UInt8, m_objectFlag);
    }

    void GLBufferManage::CleanupResources()
    {
        m_objectInfo.reset();
    }

    bool GLBufferManage::AddQuads(unsigned int gid, unsigned int objectID, const odr::Line3D& lBorder, const odr::Line3D& rBorder, QColor color)
    {
        assert(lBorder.size() == rBorder.size());
        assert(lBorder.size() >= 2);

        auto nNewVertex = (lBorder.size() - 1) * 3 * 2;
        if (m_vertexBufferCount + nNewVertex > m_vertexBufferData.size())
        {
            return false;
        }

        m_vao.bind();
        m_vbo.bind();

        std::set<GLuint> vids;
        const auto vertexBufferChangeBegin = m_vertexBufferCount;
        for (int i = 0; i < lBorder.size() - 1; ++i)
        {
            auto l0 = lBorder[i], l1 = lBorder[i + 1];
            auto r0 = rBorder[i], r1 = rBorder[i + 1];

            auto v11 = m_vertexBufferCount++;
            m_vertexBufferData[v11] = Vertex(QVector3D(l0[0], l0[1], l0[2]), color, gid, objectID);
            vids.emplace(v11);
            auto v12 = m_vertexBufferCount++;
            m_vertexBufferData[v12] = Vertex(QVector3D(r0[0], r0[1], r0[2]), color, gid, objectID);
            vids.emplace(v12);
            auto v13 = m_vertexBufferCount++;
            m_vertexBufferData[v13] = Vertex(QVector3D(l1[0], l1[1], l1[2]), color, gid, objectID);
            vids.emplace(v13);

            auto v21 = m_vertexBufferCount++;
            m_vertexBufferData[v21] = Vertex(QVector3D(l1[0], l1[1], l1[2]), color, gid, objectID);
            vids.emplace(v21);
            auto v22 = m_vertexBufferCount++;
            m_vertexBufferData[v22] = Vertex(QVector3D(r0[0], r0[1], r0[2]), color, gid, objectID);
            vids.emplace(v22);
            auto v23 = m_vertexBufferCount++;
            m_vertexBufferData[v23] = Vertex(QVector3D(r1[0], r1[1], r1[2]), color, gid, objectID);
            vids.emplace(v23);
        }
        auto ptr_v = m_vbo.mapRange(vertexBufferChangeBegin * sizeof(Vertex),
            (m_vertexBufferCount - vertexBufferChangeBegin) * sizeof(Vertex),
            QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
        assert(ptr_v != nullptr);
        memcpy(ptr_v, m_vertexBufferData.data() + vertexBufferChangeBegin,
            (m_vertexBufferCount - vertexBufferChangeBegin) * sizeof(Vertex));
        m_vbo.unmap();

        m_vbo.release();
        m_vao.release();

        idToVids.emplace(gid, vids);

        assert(vertexBufferChangeBegin + nNewVertex == m_vertexBufferCount);
        return true;
    }

    bool GLBufferManage::AddPoly(unsigned int gid, unsigned int objectID, const odr::Line3D& boundary, QColor color)
    {
        auto newTriangles = Triangulate_2_5d(boundary);
        auto nNewVertex = newTriangles.size() * 3;
        if (m_vertexBufferCount + nNewVertex > m_vertexBufferData.size())
        {
            return false;
        }

        m_vao.bind();
        m_vbo.bind();

        std::set<GLuint> vids;
        const auto vertexBufferChangeBegin = m_vertexBufferCount;
        for (auto tri : newTriangles)
        {
            auto p1 = boundary[std::get<0>(tri)];
            auto v1 = m_vertexBufferCount++;
            vids.emplace(v1);
            m_vertexBufferData[v1] = Vertex(QVector3D(p1[0], p1[1], p1[2]), color, gid, objectID);

            auto p2 = boundary[std::get<1>(tri)];
            auto v2 = m_vertexBufferCount++;
            vids.emplace(v2);
            m_vertexBufferData[v2] = Vertex(QVector3D(p2[0], p2[1], p2[2]), color, gid, objectID);

            auto p3 = boundary[std::get<2>(tri)];
            auto v3 = m_vertexBufferCount++;
            vids.emplace(v3);
            m_vertexBufferData[v3] = Vertex(QVector3D(p3[0], p3[1], p3[2]), color, gid, objectID);
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

        assert(vertexBufferChangeBegin + nNewVertex == m_vertexBufferCount);
        return true;
    }

    bool GLBufferManage::AddColumn(unsigned int gid, unsigned int objectID, const odr::Line3D& boundary, double h, QColor color)
    {
        auto capTriangles = Triangulate_2_5d(boundary);
        auto nNewVertex = capTriangles.size() * 3 * 2 + boundary.size() * 3 * 2;
        if (m_vertexBufferCount + nNewVertex > m_vertexBufferData.size())
        {
            return false;
        }

        m_vao.bind();
        m_vbo.bind();

        std::set<GLuint> vids;
        const auto vertexBufferChangeBegin = m_vertexBufferCount;
        // Top & bottom
        for (auto tri : capTriangles)
        {
            auto p1 = boundary[std::get<0>(tri)];
            auto v1 = m_vertexBufferCount++;
            vids.emplace(v1);
            m_vertexBufferData[v1] = Vertex(QVector3D(p1[0], p1[1], p1[2]), color, gid, objectID);

            auto p2 = boundary[std::get<1>(tri)];
            auto v2 = m_vertexBufferCount++;
            vids.emplace(v2);
            m_vertexBufferData[v2] = Vertex(QVector3D(p2[0], p2[1], p2[2]), color, gid, objectID);

            auto p3 = boundary[std::get<2>(tri)];
            auto v3 = m_vertexBufferCount++;
            vids.emplace(v3);
            m_vertexBufferData[v3] = Vertex(QVector3D(p3[0], p3[1], p3[2]), color, gid, objectID);

            auto v1h = m_vertexBufferCount++;
            vids.emplace(v1h);
            m_vertexBufferData[v1h] = Vertex(QVector3D(p1[0], p1[1], p1[2] + h), color, gid, objectID);

            auto v2h = m_vertexBufferCount++;
            vids.emplace(v2h);
            m_vertexBufferData[v2h] = Vertex(QVector3D(p2[0], p2[1], p2[2] + h), color, gid, objectID);

            auto v3h = m_vertexBufferCount++;
            vids.emplace(v3h);
            m_vertexBufferData[v3h] = Vertex(QVector3D(p3[0], p3[1], p3[2] + h), color, gid, objectID);
        }

        // side
        for (int i = 0; i != boundary.size(); ++i)
        {
            auto p1 = boundary[i];
            auto p2 = boundary[(i + 1) % boundary.size()];
            auto p3 = odr::Vec3D{ p1[0], p1[1], p1[2] + h };
            auto p4 = odr::Vec3D{ p2[0], p2[1], p2[2] + h };

            auto v1 = m_vertexBufferCount++;
            vids.emplace(v1);
            m_vertexBufferData[v1] = Vertex(QVector3D(p1[0], p1[1], p1[2]), color, gid, objectID);

            auto v2 = m_vertexBufferCount++;
            vids.emplace(v2);
            m_vertexBufferData[v2] = Vertex(QVector3D(p2[0], p2[1], p2[2]), color, gid, objectID);

            auto v3 = m_vertexBufferCount++;
            vids.emplace(v3);
            m_vertexBufferData[v3] = Vertex(QVector3D(p3[0], p3[1], p3[2]), color, gid, objectID);

            auto v1_1 = m_vertexBufferCount++;
            vids.emplace(v1_1);
            m_vertexBufferData[v1_1] = Vertex(QVector3D(p3[0], p3[1], p3[2]), color, gid, objectID);

            auto v2_1 = m_vertexBufferCount++;
            vids.emplace(v2_1);
            m_vertexBufferData[v2_1] = Vertex(QVector3D(p2[0], p2[1], p2[2]), color, gid, objectID);

            auto v3_1 = m_vertexBufferCount++;
            vids.emplace(v3_1);
            m_vertexBufferData[v3_1] = Vertex(QVector3D(p4[0], p4[1], p4[2]), color, gid, objectID);
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

        assert(vertexBufferChangeBegin + nNewVertex == m_vertexBufferCount);
        return true;
    }

    void GLBufferManage::UpdateItem(unsigned int objectID, uint8_t flag)
    {
        if (objectID >= MaxObjectID)
        {
            throw std::logic_error("Object ID out of range!");
        }
        m_objectFlag[objectID] = flag;
        m_objectInfo->setData(objectID,0,0,1,1,1,
            QOpenGLTexture::PixelFormat::Red, QOpenGLTexture::PixelType::UInt8, m_objectFlag + objectID);
    }

    uint8_t GLBufferManage::GetItemFlag(unsigned int objectID)
    {
        return m_objectFlag[objectID];
    }

    void GLBufferManage::UpdateObjectID(unsigned int graphicsID, unsigned int objectID)
    {
        m_vao.bind();
        m_vbo.bind();

        for (auto vid : idToVids.at(graphicsID))
        {
            m_vertexBufferData[vid].objectID = objectID;

            auto ptr_v = m_vbo.mapRange(vid * sizeof(Vertex), sizeof(Vertex),
                QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
            memcpy(ptr_v, m_vertexBufferData.data() + vid, sizeof(Vertex));
            m_vbo.unmap();
        }

        m_vbo.release();
        m_vao.release();
    }

    void GLBufferManage::RemoveItem(unsigned int gid)
    {
        if (!Road::ClearingMap)
        {
            m_vao.bind();
            m_vbo.bind();
        }
        const auto& vidsToRemove = idToVids.at(gid);
        for (auto vid_it = vidsToRemove.rbegin(); vid_it != vidsToRemove.rend(); ++vid_it)
        {
            auto vid = *vid_it;
            if (!Road::ClearingMap && m_vertexBufferCount != 0 && m_vertexBufferCount - 1 != vid)
            {
                assert(m_vertexBufferData[vid].graphicsID == gid);
                auto objectToMove = m_vertexBufferData[m_vertexBufferCount - 1].graphicsID;
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
        idToVids.erase(gid);

        if (!Road::ClearingMap)
        {
            m_vao.release();
            m_vbo.release();
        }
    }

    void GLBufferManage::RemoveObject(unsigned int objectID)
    {
        m_objectFlag[objectID] = static_cast<int>(ObjectDisplayFlag::Normal);
        m_objectInfo->setData(objectID, 0, 0, 1, 1, 1,
            QOpenGLTexture::PixelFormat::Red, QOpenGLTexture::PixelType::UInt8, m_objectFlag + objectID);
    }

    void GLBufferManage::Draw(QMatrix4x4 worldToView)
    {
        auto shaderProgramm = shader.shaderProgram();
        shaderProgramm->bind();
        shaderProgramm->setUniformValue(shader.m_uniformIDs[0], worldToView);
        m_vao.bind();
        m_objectInfo->bind(0);
        glDrawArrays(GL_TRIANGLES, 0, m_vertexBufferCount);
        m_vao.release();
        m_objectInfo->release();
        shader.shaderProgram()->release();
    }

    int GLBufferManage::Useage_pct() const
    {
        return static_cast<float>(m_vertexBufferCount) / m_vertexBufferData.size() * 100;
    }
}