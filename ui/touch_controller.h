#pragma once

#include <QTouchEvent>
#include <QList>
#include <optional>

#include "Camera.h"

namespace LM
{
    class FreeRotController
    {
    public:
        void Update(QVector2D p, Camera& camera);

    private:
        QVector3D dragRotFixedRay;
        QVector2D lastP;

        bool initialized = false;
    };

    class TouchController
    {
        // single-point touch(no p2): rotation mode (only available in View Mode)
        // dual-point touch: zoom mode | pan mode
    public:
        void Update(QList<QTouchEvent::TouchPoint> points, Camera& camera);

    private:
        enum class TouchMode
        {
            UnDetermined = 0,
            SingleTouch = 1,
            DualTouch = 2
        };

        TouchMode touchMode = TouchMode::UnDetermined;

        // SingleTouch
        FreeRotController freeRotSession;

        // DualTouch
        QQuaternion initialRotation;
        float initialHandleRotation;
        float lastHandleDist;
        QVector2D lastHandlePos;
    };
}