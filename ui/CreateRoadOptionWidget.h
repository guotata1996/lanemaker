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

    void SetOption(const RoadRunner::SectionProfile&, const RoadRunner::SectionProfile&);

signals:
    void OptionChangedByUser(RoadRunner::SectionProfile left, RoadRunner::SectionProfile right);

protected:
    void showEvent(QShowEvent* event) override;

    const int SingleSideLaneLimit = 10;

    // Force handles alignment
    void resizeEvent(QResizeEvent* event) override;

    void paintEvent(QPaintEvent* event) override;

    void mousePressEvent(QMouseEvent* event) override;

    void mouseMoveEvent(QMouseEvent* event) override;

    void mouseReleaseEvent(QMouseEvent* event) override;

    RoadRunner::SectionProfile activeLeftSetting;
    RoadRunner::SectionProfile activeRightSetting;

    float TickInterval;
    int XCenter, YCenter, XLeft, XRight;

    std::array<int, 4> handleX;
    std::vector<int> dragIndex;;
    std::vector<int> handleOrigin;
    int lOuterResult, lInnerResult, rOuterResult, rInnerResult;

    int dragOrigin;

    bool changedExternally = false;
};

class CreateLaneOptionWidget :
    public QWidget
{
    Q_OBJECT
public:
    CreateLaneOptionWidget();

    void SetOption(const RoadRunner::SectionProfile&, const RoadRunner::SectionProfile&);

protected:
    void showEvent(QShowEvent* event) override;

signals:
    void OptionChangedByUser(RoadRunner::SectionProfile left, RoadRunner::SectionProfile right);

private:
    RoadRunner::SectionProfile LeftResult() const;
    RoadRunner::SectionProfile RightResult() const;

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

    /*picking from existing*/
    void SetOption(const RoadRunner::SectionProfile&, const RoadRunner::SectionProfile&);

    void GotoRoadMode();
    void GotoLaneMode();

    RoadRunner::SectionProfile LeftResult() const { return leftProfileSetting; }
    RoadRunner::SectionProfile RightResult() const { return rightProfileSetting; }

    virtual QSize sizeHint() const override;

public slots:
    void OptionChangedOnPage(RoadRunner::SectionProfile left, RoadRunner::SectionProfile right);

private:
    CreateRoadOptionWidget* roadMode;
    CreateLaneOptionWidget* laneMode;

    RoadRunner::SectionProfile leftProfileSetting, rightProfileSetting;
};