#include "DrawOptionDialog.h"

#include "LaneConfigWidget.h"
#include "map_view_gl.h"

DrawOptionDialog::DrawOptionDialog(QWidget* parent):
    AnimatedPopupDialog(QSize(parent->width() / 3 * 2, parent->height() / 2), true, parent)
{
    QVBoxLayout* layout = new QVBoxLayout;
    laneConfig = new LaneConfigWidget(true);
    layout->addWidget(laneConfig);
    heightConfig = new QDial;
    heightConfig->setRange(-6, 6);
    heightConfig->setNotchesVisible(true);
    layout->addWidget(heightConfig, 1);
    setLayout(layout);

    heightDisplay = new QLabel("G", this);
    heightDisplay->setStyleSheet("font: bold 28px");
    heightDisplay->setAlignment(Qt::AlignHCenter);
    connect(heightConfig, &QDial::valueChanged, this, [this](int val)
        {
            QString disp = QString::number(val);
            if (val > 0)
            {
                disp = "+" + disp;
            }
            else if (val == 0)
            {
                disp = "G";
            }
            heightDisplay->setText(disp);
        });
}

void DrawOptionDialog::showEvent(QShowEvent* e)
{
    if (g_laneConfig->RoadMode())
    {
        laneConfig->GotoRoadMode();
    }
    else
    {
        laneConfig->GotoLaneMode();
    }
    laneConfig->SetOption(g_laneConfig->LeftResult(), g_laneConfig->RightResult());
    heightConfig->setValue(LM::g_createRoadElevationOption);
    heightDisplay->setGeometry(
        heightConfig->geometry().center().x() - 30, 
        heightConfig->geometry().center().y(), 
        60, 30);
    AnimatedPopupDialog::showEvent(e);
}

void DrawOptionDialog::closeEvent(QCloseEvent* e)
{
    g_laneConfig->SetOption(laneConfig->LeftResult(), laneConfig->RightResult());
    LM::g_createRoadElevationOption = heightConfig->value();
    AnimatedPopupDialog::closeEvent(e);
}