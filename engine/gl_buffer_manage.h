#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QtGui/QOpenGLExtraFunctions>
#include <vector>
#include <set>

#include "Math.hpp"
#include "Vertex.h"
#include "ShaderProgram.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

namespace
{
    typedef CGAL::Simple_cartesian<double> K;
    typedef K::FT FT;
    typedef K::Point_3 Point;
    typedef CGAL::Surface_mesh<Point> Mesh;
}

namespace RoadRunner
{
    class GLBufferManage: public QOpenGLFunctions
    {
    public:
        GLBufferManage(unsigned int capacity);
        void Initialize();

        bool AddQuads(unsigned int objectID, const odr::Line3D& lBorder, const odr::Line3D& rBorder, QColor color);
        bool AddPoly(unsigned int objectID, const odr::Line3D& boundary, QColor color);
        bool AddColumn(unsigned int objectID, const odr::Line3D& boundary, double h, QColor color);
        void UpdateItem(unsigned int, QColor);
        void RemoveItem(unsigned int);

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
    };

    class GLBufferManageInstanced : public QOpenGLExtraFunctions
    {
    public:
        GLBufferManageInstanced(unsigned int capacity);
        void Initialize();

        unsigned int AddInstance(unsigned int id, QMatrix4x4 trans);
        void UpdateInstance(unsigned int, QMatrix4x4);
        void RemoveInstance(unsigned int);

        void Draw(QMatrix4x4 worldToView);
    private:
        static Pose FromMatrix(QMatrix4x4);

        Mesh m_mesh;

        QOpenGLVertexArrayObject    m_vao;

        std::vector<Vertex>         m_vertexBufferData;
        QOpenGLBuffer				m_vertex_vbo;

        std::vector<Pose>           m_poseData;
        QOpenGLBuffer				m_instance_vbo;
        unsigned int                m_instanceCount;

        std::map<unsigned int, unsigned int> idToInstanceID;

        ShaderProgram shader;
    };
}