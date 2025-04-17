#pragma once

#include <QTouchEvent>
#include <QList>
#include <optional>

#include "Camera.h"

namespace LM
{
    class TouchController
    {
        // single-point touch(no p2): rotation mode (only available in View Mode)
        // dual-point touch: zoom mode | pan mode
    public:
        void Update(QList<QTouchEvent::TouchPoint> points, Camera& camera);

    private:
        QVector2D p1, p2;

        // touchMode == 1
        QVector3D dragRotFixedRay;
        QVector2D lastP;

        // touchMode == 2
        QQuaternion initialRotation;
        float initialHandleRotation;
        float lastHandleDist;
        QVector2D lastHandlePos;

        int touchMode = 0;
    };
}