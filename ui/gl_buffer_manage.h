#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <vector>
#include <set>

#include "Math.hpp"
#include "Vertex.h"
#include "ShaderProgram.h"

namespace RoadRunner
{
    class GLBufferManage
    {
    public:
        GLBufferManage(unsigned int capacity);
        void Initialize();

        bool AddQuads(unsigned int objectID, const odr::Line3D& lBorder, const odr::Line3D& rBorder, QColor color);
        bool AddPoly(unsigned int objectID, const odr::Line3D& boundary, QColor color);
        void UpdateItem(unsigned int, QColor);
        void RemoveItem(unsigned int);

        unsigned int Bind(QMatrix4x4 worldToView);
        void Unbind();

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
    };
}