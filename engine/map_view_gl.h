#pragma once

#include "Math.hpp"
#include "Camera.h"
#include "gl_buffer_manage.h"
#include "action_defs.h"
#include "touch_controller.h"

#include <QOpenGLWidget>
#include <qvector2d.h>
#include <QMatrix4x4>
#include <optional>
#include <map>

namespace LM
{
	const unsigned int NVehicleVariations = 3;

	class AbstractGraphicsItem;
	class PermanentGraphics;
	class InstancedGraphics;
	class UILayover;

	class MapViewGL : public QOpenGLWidget, QOpenGLExtraFunctions
	{
		friend class AbstractGraphicsItem;
		friend class PermanentGraphics;
		friend class InstancedGraphics;
		friend class UILayover;

		friend class FreeRotController;
		friend class TouchController;

		Q_OBJECT
	public:
		MapViewGL();
		void CleanupResources();
		void ResetCamera();

		// permanent objects only
		void UpdateObject(unsigned int objectID, uint8_t flag);
		void UpdateObjectID(unsigned int graphicsID, unsigned int objectID);
		uint8_t GetObjectFlag(unsigned int objectID);
		void RemoveObject(unsigned int objectID);

		void SetViewFromReplay(Transform3D t);
		void UpdateRayHit(QPoint screen, bool fromReplay=false);
		int VBufferUseage_pct() const;
		float Zoom() const;

	signals:
		void MousePerformedAction(LM::MouseAction); // excluding view adjustment / Scene button event
        void KeyPerformedAction(LM::KeyPressAction);// excluding view adjustment, including Scene button event

	protected:
		void initializeGL() override;
		void resizeGL(int width, int height) override;
		void paintGL() override;

		// Returns graphics ID. If objectID is -1, it is considered temporary.
		unsigned int AddQuads(const odr::Line3D& lBorder, const odr::Line3D& rBorder, QColor color, unsigned int objID = -1);
		unsigned int AddLine(const odr::Line3D& border, double width, QColor color, unsigned int objID = -1);
		unsigned int AddPoly(const odr::Line3D& boundary, QColor color, unsigned int objID = -1);
		unsigned int AddColumn(const odr::Line3D& boundary, double h, QColor color, unsigned int objID = -1);
		void RemoveItem(unsigned int graphicsID, bool temporary = false);

		// Instanced rendering, for traffic.
		void AddInstance(unsigned int id, QColor color, unsigned int variation);
		void UpdateInstance(unsigned int, const QMatrix4x4, unsigned int);
		void RemoveInstance(unsigned int, unsigned int);

		// Background
		unsigned int AddBackgroundLine(const odr::Line3D& line, double width, QColor color);
		void RemoveBackground(unsigned int);

		// Layover
		void AddSceneLayover(uint32_t id, odr::Vec3D scenePos, QPixmap icon, QRect lwOffset, int syntax);
		void RemoveSceneLayover(uint32_t id);

		void mousePressEvent(QMouseEvent* event) override;
		void mouseDoubleClickEvent(QMouseEvent* event) override;
		void mouseMoveEvent(QMouseEvent* event) override;
		void mouseReleaseEvent(QMouseEvent* event) override;
		void wheelEvent(QWheelEvent* event) override;

		bool event(QEvent* event) override;
		
#ifdef __linux__
		public:
#endif
		void keyPressEvent(QKeyEvent* event) override;

	private:
		static void LineToQuads(const odr::Line3D& border, double width, odr::Line3D& outLSide, odr::Line3D& outRSide);

		QPoint lastMousePos;
		bool dragPan = false;
		std::optional<FreeRotController> freeRotateSession;
		std::optional<TouchController> touchSession;
		std::optional<int> touchSessionType;

		QVector3D PointerDirection(QPoint cursor) const;

		QPointF PixelLocation(QVector3D globalDir) const;

		QVector2D PointerOnGround(QPoint cursor) const;

		std::unique_ptr<GLBufferManage>     permanentBuffer;
		std::unique_ptr<GLBufferManage>     temporaryBuffer;
		std::unique_ptr<GLBufferManage>     backgroundBuffer;
		std::array<GLBufferManageInstanced, NVehicleVariations> vehicleBuffer;

		QMatrix4x4					m_worldToView;	// cached world to view transformation matrix

		/*! The projection matrix, updated whenever the viewport geometry changes (in resizeGL() ). */
		QMatrix4x4					m_projection;
		Camera						m_camera;		// Camera position, orientation and lens data

		struct SceneTiedLayover
		{
			uint32_t id;
			odr::Vec3D pos;
			QRect lwOffset;
			QPixmap icon;
			int syntax;

			QRect renderedRect(QMatrix4x4 worldToView) const;
		};
		std::map<uint32_t, SceneTiedLayover> sceneTiedLayovers;

        bool ignoreNextMouseRelease; // for scene buttons
	};

	extern MapViewGL* g_mapViewGL;
	extern std::string g_PointerRoadID;
	extern double g_PointerRoadS;
	extern int g_PointerLane;
	extern odr::Vec2D g_PointerOnGround;
	extern odr::Vec3D g_CameraPosition;
	extern int g_createRoadElevationOption;
	extern unsigned int g_PointerVehicle;
	extern bool touchScreen;
}