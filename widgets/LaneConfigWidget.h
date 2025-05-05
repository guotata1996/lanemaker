#pragma once
#include <QtWidgets>
#include "road_profile.h"

#include <array>
#include <vector>

class CrossSectionVisual :
    public QWidget
{
    Q_OBJECT
public:
    CrossSectionVisual();

    void Reset();

    void SetOption(const LM::LanePlan&, const LM::LanePlan&);

    void SetMode(bool roadMode);
    bool IsRoadMode() const { return roadMode; }

    LM::LanePlan activeLeftSetting;
    LM::LanePlan activeRightSetting;

signals:
    void OptionChangedByUser(LM::LanePlan left, LM::LanePlan right);

protected:
    void showEvent(QShowEvent* event) override;

    const int SingleSideLaneLimit = 10;

    // Force handles alignment
    void resizeEvent(QResizeEvent* event) override;

    void paintEvent(QPaintEvent* event) override;

    void mousePressEvent(QMouseEvent* event) override;

    void mouseMoveEvent(QMouseEvent* event) override;

    void mouseReleaseEvent(QMouseEvent* event) override;

    float TickInterval;
    int XCenter, YCenter, XLeft, XRight;

    std::array<int, 4> handleX;
    std::vector<int> dragIndex;
    std::vector<int> handleOrigin;
    int lOuterResult, lInnerResult, rOuterResult, rInnerResult;

    int dragOrigin;
    int dragLimit; // absolute value

    bool roadMode;
    bool changedExternally = false;

    const QImage rightLogo, leftLogo;
};

class LaneConfigWidget :
    public QWidget
{
    Q_OBJECT
public:
    LaneConfigWidget();

    void Reset();

    /*picking from existing*/
    void SetOption(const LM::LanePlan&, const LM::LanePlan&);

    void GotoRoadMode();
    void GotoLaneMode();

    LM::LanePlan LeftResult() const { return visual->activeLeftSetting; }
    LM::LanePlan RightResult() const { return visual->activeRightSetting; }
    bool RoadMode() const { return visual->IsRoadMode(); }

    virtual QSize sizeHint() const override;

private slots:
    void OnOptionChange(LM::LanePlan left, LM::LanePlan right);

private:
    CrossSectionVisual* visual;
    QToolButton* leftMinus, * leftPlus, * rightMinus, * rightPlus;

    const QPixmap incLogo, decLogo;
};

extern LaneConfigWidget* g_laneConfig;