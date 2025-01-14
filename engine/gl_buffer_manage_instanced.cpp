#include "gl_buffer_manage.h"
#include "triangulation.h"
#include "Road.h"
#include "util.h"

#include <QOpenGLShaderProgram>
#include <CGAL/IO/OBJ.h>

namespace RoadRunner
{
    GLBufferManageInstanced::GLBufferManageInstanced(unsigned int capacity) :
        m_poseData(capacity), m_instanceCount(0),
        m_vertex_vbo(QOpenGLBuffer::VertexBuffer),
        m_instance_vbo(QOpenGLBuffer::VertexBuffer),
        shader(":/shaders/instanced.vert", ":/shaders/simple.frag")
    {
        shader.m_uniformNames.append("worldToView");

        QString tempFilePath = RoadRunner::ExtractResourceToTempFile(":/models/car.obj");

        std::ifstream input(tempFilePath.toStdString(), std::ios::binary);

        if (!input) {
            spdlog::info("Can't open the model file.");
        }
        else if (!CGAL::IO::read_OBJ(input, m_mesh))
        {
            spdlog::info("Failed to load the OBJ file.");
        }
        else {
            spdlog::info("OBJ file loaded successfully.");
            std::cout << "Number of vertices: " << m_mesh.number_of_vertices() << std::endl;
            std::cout << "Number of faces: " << m_mesh.number_of_faces() << std::endl;
        }
    }

    void GLBufferManageInstanced::Initialize()
    {
        initializeOpenGLFunctions();

        shader.create();
        m_vao.create();
        m_vertex_vbo.create();
        m_instance_vbo.create();
        m_vao.bind();
        auto shaderProgramm = shader.shaderProgram();

        // vertex buffer
        m_vertex_vbo.bind();
        m_vertex_vbo.setUsagePattern(QOpenGLBuffer::DynamicDraw); // TODO: staticdraw
        for (const auto& face : m_mesh.faces()) {
            for (auto vertex : CGAL::vertices_around_face(m_mesh.halfedge(face), m_mesh)) {
                const Point& p = m_mesh.point(vertex);
                m_vertexBufferData.push_back(Vertex(QVector3D(p.x(), p.y(), p.z()), Qt::red, 0));
            }
        }

        m_vertex_vbo.allocate(m_vertexBufferData.data(), m_vertexBufferData.size() * sizeof(Vertex));

        // vertex pos
        shaderProgramm->enableAttributeArray(0);
        shaderProgramm->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(Vertex));
        // vertex color
        shaderProgramm->enableAttributeArray(1);
        shaderProgramm->setAttributeBuffer(1, GL_FLOAT, offsetof(Vertex, r), 3, sizeof(Vertex));
        m_vertex_vbo.release();

        // instance buffer
        m_instance_vbo.bind();
        m_instance_vbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
        m_instance_vbo.allocate(m_poseData.data(), m_poseData.size() * sizeof(Pose));

        // instance pos
        shaderProgramm->enableAttributeArray(2);
        shaderProgramm->setAttributeBuffer(2, GL_FLOAT, offsetof(Pose, m00), 4, sizeof(Pose));
        glVertexAttribDivisor(2, 1);

        shaderProgramm->enableAttributeArray(3);
        shaderProgramm->setAttributeBuffer(3, GL_FLOAT, offsetof(Pose, m10), 4, sizeof(Pose));
        glVertexAttribDivisor(3, 1);
        
        shaderProgramm->enableAttributeArray(4);
        shaderProgramm->setAttributeBuffer(4, GL_FLOAT, offsetof(Pose, m20), 4, sizeof(Pose));
        glVertexAttribDivisor(4, 1);

        shaderProgramm->enableAttributeArray(5);
        shaderProgramm->setAttributeBuffer(5, GL_FLOAT, offsetof(Pose, m30), 4, sizeof(Pose));
        glVertexAttribDivisor(5, 1);

        m_instance_vbo.release();
        m_vao.release();
    }

    unsigned int GLBufferManageInstanced::AddInstance(unsigned int id, QMatrix4x4 trans)
    {
        m_vao.bind();
        m_instance_vbo.bind();
        trans = trans.transposed();
        m_poseData[m_instanceCount] = FromMatrix(trans);
        m_poseData[m_instanceCount].objectID = id;

        auto ptr = m_instance_vbo.mapRange(m_instanceCount * sizeof(Pose),
            sizeof(Pose), QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
        memcpy(ptr, m_poseData.data() + m_instanceCount, sizeof(Pose));
        m_instance_vbo.unmap();
        m_instance_vbo.release();
        idToInstanceID.emplace(id, m_instanceCount);
        m_instanceCount++;
        m_vao.release();
        return m_instanceCount - 1;
    }

    void GLBufferManageInstanced::UpdateInstance(unsigned int id, QMatrix4x4 trans)
    {
        m_vao.bind();
        m_instance_vbo.bind();

        auto instanceID = idToInstanceID.at(id);
        m_poseData[instanceID] = FromMatrix(trans);
        m_poseData[instanceID].objectID = id;

        auto ptr = m_instance_vbo.mapRange(instanceID * sizeof(Pose),
            sizeof(Pose), QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
        memcpy(ptr, m_poseData.data() + instanceID, sizeof(Pose));
        m_instance_vbo.unmap();
        m_instance_vbo.release();
        m_vao.release();
    }

    void GLBufferManageInstanced::RemoveInstance(unsigned int id)
    {
        auto instanceID = idToInstanceID.at(id);
        idToInstanceID.erase(id);
        if (instanceID != m_instanceCount - 1)
        {
            auto lastPose = m_poseData[m_instanceCount - 1];
            assert(idToInstanceID[lastPose.objectID] == m_instanceCount - 1);
            idToInstanceID[lastPose.objectID] = instanceID;
            m_poseData[instanceID] = lastPose;

            m_vao.bind();
            m_instance_vbo.bind();

            auto ptr = m_instance_vbo.mapRange(instanceID * sizeof(Pose),
                sizeof(Pose), QOpenGLBuffer::RangeInvalidate | QOpenGLBuffer::RangeWrite);
            memcpy(ptr, m_poseData.data() + instanceID, sizeof(Pose));
            m_instance_vbo.unmap();
            m_instance_vbo.release();
            m_vao.release();
        }
        m_instanceCount--;
    }

    void GLBufferManageInstanced::Draw(QMatrix4x4 worldToView)
    {
        auto shaderProgramm = shader.shaderProgram();
        shaderProgramm->bind();
        shaderProgramm->setUniformValue(shader.m_uniformIDs[0], worldToView);

        m_vao.bind();
        glDrawArraysInstanced(GL_TRIANGLES, 0, m_vertexBufferData.size(), m_instanceCount);
        m_vao.release();
        shader.shaderProgram()->release();
    }

    Pose GLBufferManageInstanced::FromMatrix(QMatrix4x4 trans)
    {
        Pose rtn;
        trans = trans.transposed();

        rtn.m00 = trans.row(0).x();
        rtn.m01 = trans.row(0).y();
        rtn.m02 = trans.row(0).z();
        rtn.m03 = trans.row(0).w();

        rtn.m10 = trans.row(1).x();
        rtn.m11 = trans.row(1).y();
        rtn.m12 = trans.row(1).z();
        rtn.m13 = trans.row(1).w();

        rtn.m20 = trans.row(2).x();
        rtn.m21 = trans.row(2).y();
        rtn.m22 = trans.row(2).z();
        rtn.m23 = trans.row(2).w();

        rtn.m30 = trans.row(3).x();
        rtn.m31 = trans.row(3).y();
        rtn.m32 = trans.row(3).z();
        rtn.m33 = trans.row(3).w();
        return rtn;
    }
}