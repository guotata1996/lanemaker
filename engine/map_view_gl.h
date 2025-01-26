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
	const unsigned int NVehicleVariations = 3;

	class MapViewGL : public OpenGLWindow
	{
		Q_OBJECT
	public:
		MapViewGL();
		void ResetCamera();

		// Returns graphics ID. If objectID is -1, it is considered temporary.
		unsigned int AddQuads(const odr::Line3D& lBorder, const odr::Line3D& rBorder, QColor color, unsigned int objID = -1);
		unsigned int AddLine(const odr::Line3D& border, double width, QColor color, unsigned int objID = -1);
		unsigned int AddPoly(const odr::Line3D& boundary, QColor color, unsigned int objID = -1);
		unsigned int AddColumn(const odr::Line3D& boundary, double h, QColor color, unsigned int objID = -1);
		void UpdateItem(unsigned int objectID, uint8_t);  // permanent objects only
		uint8_t GetItemFlag(unsigned int objectID);
		void UpdateObjectID(unsigned int graphicsID, unsigned int objectID);  // permanent objects only
		void RemoveItem(unsigned int graphicsID, bool temporary = false);
		void RemoveObject(unsigned int objectID);

		// Instanced rendering, for traffic.
		void AddInstance(unsigned int id, QColor color, unsigned int variation);
		void UpdateInstance(unsigned int, const QMatrix4x4, unsigned int);
		void RemoveInstance(unsigned int, unsigned int);

		// Background
		unsigned int AddBackgroundLine(const odr::Line3D& line, double width, QColor color);
		void RemoveBackground(unsigned int);

		void SetViewFromReplay(Transform3D t);
		void UpdateRayHit(QPoint screen, bool fromReplay=false);
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

	private:
		static void LineToQuads(const odr::Line3D& border, double width, odr::Line3D& outLSide, odr::Line3D& outRSide);

		QPoint lastMousePos;
		std::optional<QVector3D> dragRotFixedRay;
		bool dragPan = false;

		QVector3D PointerDirection(QPoint cursor) const;

		QPointF PixelLocation(QVector3D globalDir) const;

		QVector2D PointerOnGround(QPoint cursor) const;

		GLBufferManage     permanentBuffer;
		GLBufferManage     temporaryBuffer;
		GLBufferManage     backgroundBuffer;
		std::array<GLBufferManageInstanced, NVehicleVariations> vehicleBuffer;

		QMatrix4x4					m_worldToView;	// cached world to view transformation matrix

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
	extern odr::Vec3D g_CameraPosition;
	extern int g_createRoadElevationOption;
}