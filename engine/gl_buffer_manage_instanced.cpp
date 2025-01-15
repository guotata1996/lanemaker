#include "gl_buffer_manage.h"
#include "triangulation.h"
#include "Road.h"
#include "util.h"

#include "OBJ_Loader.h"
#include <QOpenGLShaderProgram>


namespace RoadRunner
{
    objl::Loader                m_mesh;

    GLBufferManageInstanced::GLBufferManageInstanced(unsigned int capacity) :
        m_poseData(capacity), m_instanceCount(0),
        m_vertex_vbo(QOpenGLBuffer::VertexBuffer),
        m_instance_vbo(QOpenGLBuffer::VertexBuffer),
        shader(":/shaders/instanced.vert", ":/shaders/simple.frag")
    {
        shader.m_uniformNames.append("worldToView");

        QString tempFilePath = RoadRunner::ExtractResourceToTempFile(":/models/car.obj");
        bool loadout = m_mesh.LoadFile(tempFilePath.toStdString());
        assert(loadout);
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

        for (int i = 0; i < m_mesh.LoadedMeshes.size(); i++)
        {
            objl::Mesh curMesh = m_mesh.LoadedMeshes[i];
            for (int j = 0; j < curMesh.Indices.size(); j += 3)
            {
                auto ind1 = curMesh.Indices[j];
                auto ind2 = curMesh.Indices[j + 1];
                auto ind3 = curMesh.Indices[j + 2];
                auto pos1 = curMesh.Vertices[ind1].Position;
                auto pos2 = curMesh.Vertices[ind2].Position;
                auto pos3 = curMesh.Vertices[ind3].Position;
                m_vertexBufferData.push_back(Vertex(QVector3D(pos1.X, pos1.Y, pos1.Z), Qt::white, 0));
                m_vertexBufferData.push_back(Vertex(QVector3D(pos2.X, pos2.Y, pos2.Z), Qt::white, 0));
                m_vertexBufferData.push_back(Vertex(QVector3D(pos3.X, pos3.Y, pos3.Z), Qt::white, 0));
            }
        }

        m_vertex_vbo.allocate(m_vertexBufferData.data(), m_vertexBufferData.size() * sizeof(Vertex));

        // vertex pos
        shaderProgramm->enableAttributeArray(0);
        shaderProgramm->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(Vertex));
        m_vertex_vbo.release();

        // instance buffer
        m_instance_vbo.bind();
        m_instance_vbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
        m_instance_vbo.allocate(m_poseData.data(), m_poseData.size() * sizeof(Pose));

        // instance color
        shaderProgramm->enableAttributeArray(1);
        shaderProgramm->setAttributeBuffer(1, GL_FLOAT, offsetof(Pose, r), 3, sizeof(Pose));
        glVertexAttribDivisor(1, 1);

        // instance transform 4x4
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

    unsigned int GLBufferManageInstanced::AddInstance(unsigned int id, QMatrix4x4 trans, QColor color)
    {
        m_vao.bind();
        m_instance_vbo.bind();
        trans = trans.transposed();
        FromMatrix(m_poseData[m_instanceCount], trans);
        m_poseData[m_instanceCount].r = color.redF();
        m_poseData[m_instanceCount].g = color.greenF();
        m_poseData[m_instanceCount].b = color.blueF();
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
        FromMatrix(m_poseData[instanceID], trans);

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

    void GLBufferManageInstanced::FromMatrix(Pose& rtn, QMatrix4x4 trans)
    {
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
    }
}