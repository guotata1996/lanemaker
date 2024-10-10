#pragma once
#include <QtWidgets>
#include "road_profile.h"

#include <array>
#include <vector>

class CreateRoadOptionWidget :
    public QWidget
{
    Q_OBJECT
public:
    CreateRoadOptionWidget();

    void Reset();

    void SetOption(const RoadRunner::LanePlan&, const RoadRunner::LanePlan&);

    void SetMode(bool rodeMode);

    RoadRunner::LanePlan activeLeftSetting;
    RoadRunner::LanePlan activeRightSetting;

signals:
    void OptionChangedByUser(RoadRunner::LanePlan left, RoadRunner::LanePlan right);

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

    bool changedExternally = false;
    bool roadMode;

    const QImage rightLogo, leftLogo;
};

class SectionProfileConfigWidget :
    public QWidget
{
    Q_OBJECT
public:
    SectionProfileConfigWidget();

    void Reset();

    /*picking from existing*/
    void SetOption(const RoadRunner::LanePlan&, const RoadRunner::LanePlan&);

    void GotoRoadMode();
    void GotoLaneMode();

    RoadRunner::LanePlan LeftResult() const { return visual->activeLeftSetting; }
    RoadRunner::LanePlan RightResult() const { return visual->activeRightSetting; }

    virtual QSize sizeHint() const override;

public slots:
    void OptionChangedOnPage(RoadRunner::LanePlan left, RoadRunner::LanePlan right);

private:
    CreateRoadOptionWidget* visual;
    QToolButton* leftMinus, * leftPlus, * rightMinus, * rightPlus;

    const QPixmap incLogo, decLogo;
};