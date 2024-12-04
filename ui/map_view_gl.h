#pragma once

#include "OpenGLWindow.h"
#include "Math.hpp"
#include "ShaderProgram.h"
#include "Vertex.h"
#include "Camera.h"

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>

namespace RoadRunner
{
	class MapViewGL : public OpenGLWindow
	{
	public:
		MapViewGL();

		unsigned int AddQuads(const odr::Line3D& lBorder, const odr::Line3D& rBorder);

		void RemoveItem(unsigned int);

	protected:
		void initializeGL() override;
		void resizeGL(int width, int height) override;
		void paintGL() override;

		void mouseMoveEvent(QMouseEvent* event) override;

	private:
		QVector3D PointerDirection(QPoint cursor) const;

		ShaderProgram shader;

		std::array<Vertex, 2 * 262144>			m_vertexBufferData;
		std::array<GLuint, 3 * 262144>			m_elementBufferData;

		/*! Wraps an OpenGL VertexArrayObject, that references the vertex coordinates and color buffers. */
		//QOpenGLVertexArrayObject	m_vao;
		QOpenGLVertexArrayObject m_vao;

		/*! Holds position and colors in a single buffer. */
		QOpenGLBuffer				m_vbo;
		/*! Holds elements. */
		QOpenGLBuffer				m_ebo;

		/*! The projection matrix, updated whenever the viewport geometry changes (in resizeGL() ). */
		QMatrix4x4					m_projection;
		Transform3D					m_transform;	// world transformation matrix generator
		Camera						m_camera;		// Camera position, orientation and lens data
		QMatrix4x4					m_worldToView;	// cached world to view transformation matrix

		std::map<unsigned int, std::vector<unsigned int>> idToEids;
		std::map<unsigned int, std::vector<unsigned int>> idToVids;
	};

	extern MapViewGL* g_mapViewGL;
}