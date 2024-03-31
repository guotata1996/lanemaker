#include "CreateRoadOptionWidget.h"
#include <QVBoxLayout>
#include <qstyle.h>
#include <qpainter.h>

#include <spdlog/spdlog.h>

RoadRunner::SectionProfile activeLeftSetting;
RoadRunner::SectionProfile activeRightSetting;

CreateRoadOptionWidget::CreateRoadOptionWidget(QWidget* parent) : QWidget(parent)
{
    auto mainLayout = new QVBoxLayout;
    slider1 = new QSlider(Qt::Orientation::Horizontal, this);
    slider2 = new QSlider(Qt::Orientation::Horizontal, this);
    slider3 = new QSlider(Qt::Orientation::Horizontal, this);
    slider4 = new QSlider(Qt::Orientation::Horizontal, this);

    for (QSlider* silder : { slider1 , slider2, slider3, slider4 })
    {
        silder->setStyleSheet("QSlider::groove:horizontal { background-color:transparent; }"
            "QSlider::handle:horizontal {background-color:black; height:4px; width: 10px; }");

        connect(silder, &QSlider::valueChanged, this, &CreateRoadOptionWidget::sliderMoved);
    }

    resultSlider = new QSlider(Qt::Orientation::Horizontal, this);
    resultSlider->setStyleSheet("QSlider::handle:horizontal {background-color:transparent; height:0px; width: 0px; }");

    for (QSlider* silder : { slider1 , slider2, resultSlider, slider3, slider4 })
    {
        silder->setRange(-8, 8);
        silder->setTickInterval(1);
        silder->setTickPosition(QSlider::TickPosition::TicksAbove);
        silder->setFixedHeight(12);
    }

    mainLayout->addWidget(slider1);
    mainLayout->addWidget(slider2);
    mainLayout->addWidget(resultSlider);
    mainLayout->addWidget(slider3);
    mainLayout->addWidget(slider4);

    setLayout(mainLayout);

    // Default to 1 lane on left & 1 lane on right
    slider1->setValue(-2);
    slider4->setValue(2);
    sliderMoved();
}

void CreateRoadOptionWidget::sliderMoved()
{
    std::vector<int> fourVals = {
        slider1->value(),
        slider2->value(),
        slider3->value(),
        slider4->value(),
    };
    std::sort(fourVals.begin(), fourVals.end());
    int l2 = fourVals[0];
    int l1 = fourVals[1];
    lResult.laneCount = (l1 - l2) / 2;
    lResult.offsetx2 = -l1;

    int r1 = fourVals[2];
    int r2 = fourVals[3];
    rResult.laneCount = (r2 - r1) / 2;
    rResult.offsetx2 = -r1;

    update();

    if (lResult.laneCount != 0 || rResult.laneCount != 0)
    {
        activeLeftSetting = lResult;
        activeRightSetting = rResult;
    }
}

void CreateRoadOptionWidget::paintEvent(QPaintEvent* event)
{
    QWidget::paintEvent(event);

    QPainter painter(this);
    painter.setPen(Qt::NoPen);
    const int RectGap = 2;

    int y = resultSlider->y();
    int rectHeight = resultSlider->height() / 2;
    int w1 = slider1->width();
    int w2 = width();
    int margin = (w2 - w1) / 2;

    painter.setBrush(QBrush(Qt::red));
    for (int lane = 0; lane != lResult.laneCount; ++lane)
    {
        int l2 = QStyle::sliderPositionFromValue(-8, 8, -lResult.offsetx2 - lane * 2, slider1->width());
        int l1 = QStyle::sliderPositionFromValue(-8, 8, -lResult.offsetx2 - (lane + 1) * 2, slider2->width());
        int leftBegin = margin + l1;

        painter.drawRect(leftBegin + RectGap, y + rectHeight, l2 - l1 - RectGap * 2, rectHeight);
    }

    painter.setBrush(QBrush(Qt::green));
    for (int lane = 0; lane != rResult.laneCount; ++lane)
    {
        int r1 = QStyle::sliderPositionFromValue(-8, 8, -rResult.offsetx2 + lane * 2, slider1->width());
        int r2 = QStyle::sliderPositionFromValue(-8, 8, -rResult.offsetx2 + (lane + 1) * 2, slider1->width());
        int rightBegin = margin + r1;
        painter.drawRect(rightBegin + RectGap, y + rectHeight, r2 - r1 - RectGap * 2, rectHeight);
    }


}