#pragma once

#include "OpenGLWindow.h"
#include "Math.hpp"
#include "Camera.h"
#include "gl_buffer_manage.h"
#include "action_defs.h"

#include <qvector2d.h>
#include <QMatrix4x4>
#include <optional>

namespace RoadRunner
{
	class MapViewGL : public OpenGLWindow
	{
		Q_OBJECT
	public:
		MapViewGL();
		void ResetCamera();

		unsigned int AddQuads(const odr::Line3D& lBorder, const odr::Line3D& rBorder, QColor color, bool temporary=false);
		unsigned int AddPoly(const odr::Line3D& boundary, QColor color, bool temporary=false);
		void SetViewFromReplay(Transform3D t);
		void UpdateRayHit(QPoint screen);
		
		void UpdateItem(unsigned int, QColor, bool temporary = false);
		void RemoveItem(unsigned int, bool temporary = false);

		int VBufferUseage_pct() const;
		float Zoom() const;

		QMatrix4x4					m_worldToView;	// cached world to view transformation matrix

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

		GLBufferManage     permanentBuffer;
		GLBufferManage     temporaryBuffer;

		/*! The projection matrix, updated whenever the viewport geometry changes (in resizeGL() ). */
		QMatrix4x4					m_projection;
		Transform3D					m_transform;	// world transformation matrix generator
		Camera						m_camera;		// Camera position, orientation and lens data
	};

	extern MapViewGL* g_mapViewGL;
	extern std::string g_PointerRoadID;
	extern double g_PointerRoadS;
	extern int g_PointerLane;
	extern odr::Vec2D g_PointerOnGround;
}