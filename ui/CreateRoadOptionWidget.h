#pragma once
#include <qwidget.h>
#include "road_profile.h"
#include <qslider.h>

class CreateRoadOptionWidget :
    public QWidget
{
    Q_OBJECT
public:
    CreateRoadOptionWidget(QWidget* parent = nullptr);

    RoadRunner::SectionProfile lResult;
    RoadRunner::SectionProfile rResult;

protected:
    void paintEvent(QPaintEvent* event) override;

private slots:
    void sliderMoved();
private:
    QSlider* slider1;
    QSlider* slider2;
    QSlider* resultSlider;
    QSlider* slider3;
    QSlider* slider4;
};

