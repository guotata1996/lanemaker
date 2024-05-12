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
    RoadRunner::SectionProfile LeftResult() const;
    RoadRunner::SectionProfile RightResult() const;

    void SetOption(const RoadRunner::SectionProfile&, const RoadRunner::SectionProfile&);

protected:
    const int SingleSideLaneLimit = 10;

    virtual QSize sizeHint() const override;

    void paintEvent(QPaintEvent* event) override;

    void mousePressEvent(QMouseEvent* event) override;

    void mouseMoveEvent(QMouseEvent* event) override;

    void mouseReleaseEvent(QMouseEvent* event) override;

    RoadRunner::SectionProfile activeLeftSetting{ 1, 1 };
    RoadRunner::SectionProfile activeRightSetting{ -1, 1 };

    float TickInterval;
    int XCenter, YCenter, XLeft, XRight;

    std::array<int, 4> handleX;
    int dragIndex = -1;
};

