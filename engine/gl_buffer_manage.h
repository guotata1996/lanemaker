#pragma once

#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QtGui/QOpenGLExtraFunctions>
#include <QOpenGLTexture>
#include <vector>
#include <set>
#include <memory>

#include "Math.hpp"
#include "Vertex.h"
#include "ShaderProgram.h"
#include "constants.h"

namespace objl
{
    class Loader;
}

namespace LM
{
    enum class ObjectDisplayFlag
    {
        Normal = 0,
        Highlighted = 16,
        Hidden = 32,
        GreenLight = 64
    };

    class GLBufferManage: public QOpenGLFunctions
    {
    public:
        GLBufferManage(unsigned int capacity);
        void Initialize();
        void Reset();
        void CleanupResources();

        bool AddQuads(unsigned int graphicsID, unsigned int objectID, const odr::Line3D& lBorder, const odr::Line3D& rBorder, QColor color);
        bool AddPoly(unsigned int graphicsID, unsigned int objectID, const odr::Line3D& boundary, QColor color);
        bool AddColumn(unsigned int graphicsID, unsigned int objectID, const odr::Line3D& boundary, double h, QColor color);
        void UpdateItem(unsigned int objectID, uint8_t);
        uint8_t GetItemFlag(unsigned int objectID);
        void UpdateObjectID(unsigned int graphicsID, unsigned int objectID);
        void RemoveItem(unsigned int graphicsID);
        void RemoveObject(unsigned int objectID); // call after all of its items have been removed

        void Draw(QMatrix4x4 worldToView);

        int Useage_pct() const;

    private:
        std::vector<Vertex>          m_vertexBufferData;
        unsigned int                 m_vertexBufferCount;

        /*! Wraps an OpenGL VertexArrayObject, that references the vertex coordinates and color buffers. */
        QOpenGLVertexArrayObject    m_vao;

        /*! Holds position and colors in a single buffer. */
        QOpenGLBuffer				m_vbo;

        std::map<unsigned int, std::set<unsigned int>> idToVids;

        ShaderProgram shader;
        uint8_t                     m_objectFlag[MaxObjectID];
        std::unique_ptr<QOpenGLTexture> m_objectInfo;
    };

    class GLBufferManageInstanced : public QOpenGLExtraFunctions
    {
    public:
        GLBufferManageInstanced(QString modelPath, QString texPath, unsigned int capacity);
        void Initialize();
        void CleanupResources();

        unsigned int AddInstance(unsigned int id, QMatrix4x4 trans, QColor color);
        void UpdateInstance(unsigned int, QMatrix4x4);
        void RemoveInstance(unsigned int);

        void Draw(QMatrix4x4 worldToView);
    private:
        static void FromMatrix(Pose&, QMatrix4x4);

        QOpenGLVertexArrayObject    m_vao;

        std::vector<VertexInstanced>m_vertexBufferData;
        QOpenGLBuffer				m_vertex_vbo;

        std::vector<Pose>           m_poseData;
        QOpenGLBuffer				m_instance_vbo;
        unsigned int                m_instanceCount;
        bool                        pendingSend;  // batch copy all pose data to GPU

        std::map<unsigned int, unsigned int> idToInstanceID;

        ShaderProgram shader;
        objl::Loader*               m_mesh;
        std::unique_ptr<QOpenGLTexture> m_texture;
        const QString               modelPath, texturePath;
    };
}