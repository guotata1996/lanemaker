#include "DrawOptionDialog.h"

#include "LaneConfigWidget.h"
#include "map_view_gl.h"

DrawOptionDialog::DrawOptionDialog(QWidget* parent):
    AnimatedPopupDialog(QSize(parent->width() / 3 * 2, parent->height() / 2), true, parent)
{
    QVBoxLayout* layout = new QVBoxLayout;
    laneConfig = new LaneConfigWidget;
    layout->addWidget(laneConfig);
    heightConfig = new QSlider(Qt::Horizontal);
    heightConfig->setRange(-6, 6);
    layout->addWidget(heightConfig);
    setLayout(layout);
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
    AnimatedPopupDialog::showEvent(e);
}

void DrawOptionDialog::closeEvent(QCloseEvent* e)
{
    g_laneConfig->SetOption(laneConfig->LeftResult(), laneConfig->RightResult());
    LM::g_createRoadElevationOption = heightConfig->value();
    AnimatedPopupDialog::closeEvent(e);
}