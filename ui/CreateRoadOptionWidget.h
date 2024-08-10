#pragma once
#include <qwidget.h>
#include <qstackedwidget.h>
#include "road_profile.h"
#include <qslider.h>
#include <qlabel.h>

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

    RoadRunner::LanePlan activeLeftSetting;
    RoadRunner::LanePlan activeRightSetting;

    float TickInterval;
    int XCenter, YCenter, XLeft, XRight;

    std::array<int, 4> handleX;
    std::vector<int> dragIndex;;
    std::vector<int> handleOrigin;
    int lOuterResult, lInnerResult, rOuterResult, rInnerResult;

    int dragOrigin;

    bool changedExternally = false;

    const QImage rightLogo, leftLogo;
};

class CreateLaneOptionWidget :
    public QWidget
{
    Q_OBJECT
public:
    CreateLaneOptionWidget();

    void Reset();

    void SetOption(const RoadRunner::LanePlan&, const RoadRunner::LanePlan&);

protected:
    void showEvent(QShowEvent* event) override;

signals:
    void OptionChangedByUser(RoadRunner::LanePlan left, RoadRunner::LanePlan right);

private:
    RoadRunner::LanePlan LeftResult() const;
    RoadRunner::LanePlan RightResult() const;

    QSlider* leftSlider, * rightSlider;
    QLabel* resultLabel;

private slots:
    void updateLabel();
};


class SectionProfileConfigWidget :
    public QStackedWidget
{
    Q_OBJECT
public:
    SectionProfileConfigWidget();

    void Reset();

    /*picking from existing*/
    void SetOption(const RoadRunner::LanePlan&, const RoadRunner::LanePlan&);

    void GotoRoadMode();
    void GotoLaneMode();

    RoadRunner::LanePlan LeftResult() const { return leftProfileSetting; }
    RoadRunner::LanePlan RightResult() const { return rightProfileSetting; }

    virtual QSize sizeHint() const override;

public slots:
    void OptionChangedOnPage(RoadRunner::LanePlan left, RoadRunner::LanePlan right);

private:
    CreateRoadOptionWidget* roadMode;
    CreateLaneOptionWidget* laneMode;

    RoadRunner::LanePlan leftProfileSetting, rightProfileSetting;
};