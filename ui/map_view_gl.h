#pragma once

#include "OpenGLWindow.h"
#include "Math.hpp"
#include "ShaderProgram.h"
#include "Vertex.h"
#include "Camera.h"

#include <qvector2d.h>
#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <optional>

#include "action_defs.h"

namespace RoadRunner
{
	class MapViewGL : public OpenGLWindow
	{
		Q_OBJECT
	public:
		MapViewGL();

		unsigned int AddQuads(const odr::Line3D& lBorder, const odr::Line3D& rBorder, QColor color);
		unsigned int AddPoly(const odr::Line3D& boundary, QColor color);
		
		void UpdateItem(unsigned int, QColor);
		void RemoveItem(unsigned int);

		int VBufferUseage_pct() const;
		float Zoom() const;
	signals:
		void MousePerformedAction(RoadRunner::MouseAction);
		void KeyPerformedAction(RoadRunner::KeyPressAction);

	protected:
		void initializeGL() override;
		void resizeGL(int width, int height) override;
		void paintGL() override;

		void mousePressEvent(QMouseEvent* event) override;
		void mouseDoubleClickEvent(QMouseEvent* event) override;
		void mouseMoveEvent(QMouseEvent* event) override;
		void mouseReleaseEvent(QMouseEvent* event) override;

		void wheelEvent(QWheelEvent* event) override;

		void keyPressEvent(QKeyEvent* event) override;
		void keyReleaseEvent(QKeyEvent* event) override;

	private:
		QPoint lastMousePos;
		std::optional<QVector3D> dragRotFixedRay;
		bool dragPan = false;
		bool pressedKeys[Qt::Key_Z];

		QVector3D PointerDirection(QPoint cursor) const;

		QPointF PixelLocation(QVector3D globalDir) const;

		QVector2D PointerOnGround(QPoint cursor) const;

		ShaderProgram shader;

		std::array<Vertex, (1<<20)> m_vertexBufferData;
		unsigned int                m_vertexBufferCount;

		/*! Wraps an OpenGL VertexArrayObject, that references the vertex coordinates and color buffers. */
		//QOpenGLVertexArrayObject	m_vao;
		QOpenGLVertexArrayObject m_vao;

		/*! Holds position and colors in a single buffer. */
		QOpenGLBuffer				m_vbo;

		/*! The projection matrix, updated whenever the viewport geometry changes (in resizeGL() ). */
		QMatrix4x4					m_projection;
		Transform3D					m_transform;	// world transformation matrix generator
		Camera						m_camera;		// Camera position, orientation and lens data
		QMatrix4x4					m_worldToView;	// cached world to view transformation matrix

		std::map<unsigned int, std::set<unsigned int>> idToVids;
	};

	extern MapViewGL* g_mapViewGL;
	extern std::string g_PointerRoadID;
	extern double g_PointerRoadS;
	extern int g_PointerLane;
	extern odr::Vec2D g_PointerOnGround;
}