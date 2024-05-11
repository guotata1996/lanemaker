#pragma once
#include <qwidget.h>
#include "road_profile.h"
#include <qslider.h>

#include <array>

class CreateRoadOptionWidget :
    public QWidget
{
    Q_OBJECT
public:
    RoadRunner::SectionProfile lResult;
    RoadRunner::SectionProfile rResult;

    virtual QSize sizeHint() const override;

protected:
    const int SingleSideLaneLimit = 10;

    void paintEvent(QPaintEvent* event) override;

    void mousePressEvent(QMouseEvent* event) override;

    void mouseMoveEvent(QMouseEvent* event) override;

    void mouseReleaseEvent(QMouseEvent* event) override;

    float TickInterval;
    int XCenter, YCenter, XLeft, XRight;

    std::array<int, 4> handleX;
    int dragIndex = -1;
};

