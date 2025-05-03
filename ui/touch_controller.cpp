#include "touch_controller.h"
#include "map_view_gl.h"

#include "action_manager.h"

#include <spdlog/spdlog.h>

namespace LM
{
    void TouchController::Update(QList<QTouchEvent::TouchPoint> points, Camera& camera)
    {
        if (touchMode != TouchMode::UnDetermined && static_cast<int>(touchMode) != points.size())
        {
            touchMode = TouchMode::UnDetermined;
        }

        QVector2D p1, p2;

        if (points.size() >= 1)
            p1 = QVector2D(points[0].pos());
        if (points.size() >= 2)
            p2 = QVector2D(points[1].pos());

        if (touchMode == TouchMode::UnDetermined)
        {
            if (points.size() == 1)
            {
                touchMode = TouchMode::SingleTouch;
            }
            else if (points.size() == 2)
            {
                touchMode = TouchMode::DualTouch;

                initialRotation = camera.rotation();
                initialHandleRotation = std::atan2(p2.y() - p1.y(), p2.x() - p1.x());
                lastHandleDist = p1.distanceToPoint(p2);
                lastHandlePos = (p1 + p2) / 2;
            }
        }

        if (touchMode == TouchMode::SingleTouch)
        {
            freeRotSession.Update(p1, camera);
        }
        else if (touchMode == TouchMode::DualTouch)
        {
            // Rotate
            auto handleRotation = std::atan2(p2.y() - p1.y(), p2.x() - p1.x());
            auto deltaRot = QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), (handleRotation - initialHandleRotation) * 180 / M_PI);
            camera.setRotation(deltaRot * initialRotation);

            // Zoom
            auto currHandleDist = p1.distanceToPoint(p2);
            auto pm = (p1 + p2) / 2;
            auto pDirection = g_mapViewGL->PointerDirection(pm.toPoint());
            auto zoomDegree = currHandleDist / lastHandleDist - 1;
            camera.translate(zoomDegree * 100 * pDirection);
            lastHandleDist = currHandleDist;

            // Pan
            auto currGroundPos = g_mapViewGL->PointerOnGround(pm.toPoint());
            auto lastGroundPos = g_mapViewGL->PointerOnGround(lastHandlePos.toPoint());
            camera.translate(lastGroundPos - currGroundPos);
            lastHandlePos = pm;
        }
    }

    void FreeRotController::Update(QVector2D p, Camera& camera)
    {
        if (!initialized)
        {
            dragRotFixedRay = g_mapViewGL->PointerDirection(p.toPoint());
            initialized = true;
            return;
        }
        
        int i;
        const int MaxIter = 250;
        const auto backupRotation = camera.rotation();
        for (i = 0; i != MaxIter; ++i)
        {
            double step = i < MaxIter / 2 ? 1 : 0.1;
            int tol = i < MaxIter / 2 ? 1 : 3;
            QPointF rayPixel = g_mapViewGL->PixelLocation(dragRotFixedRay);

            auto xError = std::abs(p.x() - rayPixel.x());
            auto yError = std::abs(p.y() - rayPixel.y());

            if (xError <= tol && yError <= tol)
            {
                break;
            }
            if (xError > yError)
            {
                if (p.x() > rayPixel.x() + tol)
                {
                    camera.rotate(step, QVector3D(0, 0, 1));
                }
                else
                {
                    camera.rotate(-step, QVector3D(0, 0, 1));
                }
            }
            else
            {
                if (p.y() > rayPixel.y() + tol)
                {
                    camera.rotate(step, camera.right());
                }
                else
                {
                    camera.rotate(-step, camera.right());
                }
            }
        }

        auto rotated = backupRotation.conjugate()* camera.rotation();
        auto rotatedAngle = 2 * std::acos(rotated.scalar());
        if (i == MaxIter || rotatedAngle > 0.3)
        {
            camera.setRotation(backupRotation);
            auto dx = static_cast<float>(p.x() - lastP.x());
            auto dy = static_cast<float>(p.y() - lastP.y());
            dx = dx / g_mapViewGL->width() * 480;
            dy = dy / g_mapViewGL->height() * 120;

            camera.rotate(dx, QVector3D(0, 0, 1));
            camera.rotate(dy, camera.right());
        }

        if (!camera.isRotationAllowed())
        {
            camera.setRotation(backupRotation);
        }
                
        lastP = p;
    }
}