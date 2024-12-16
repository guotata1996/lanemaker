#pragma once
#include <QFrame>
#include <QGraphicsView>

#include "action_defs.h"

QT_BEGIN_NAMESPACE
class QLabel;
class QSlider;
class QToolButton;
class QButtonGroup;
class QPixmap;
QT_END_NAMESPACE

class MapView;
class RoadDrawingSession;
namespace RoadRunner
{
    class MapViewGL;
}

class SectionProfileConfigWidget;

class MainWidget : public QFrame
{
    Q_OBJECT
public:
    explicit MainWidget(QGraphicsScene* scene, QWidget* parent = nullptr);

    QGraphicsView* view() const;

    void PostEditActions();

    void SetHovering(QString);

    void SetBackgroundImage(const QPixmap& image);

    void Painted();

    void Reset();

    void RecordViewTransform();
    void SetViewFromReplay(double zoomSliderVal, double rotateSliderVal);
    void SetModeFromReplay(int mode);
    void SetElevationFromReplay(int8_t elevationSetting);

    RoadRunner::MapViewGL* mapViewGL;

signals:
    void HoveringChanged(QString);

    void FPSChanged(QString);

    void InReadOnlyMode(bool);

public slots:
    void toggleAntialiasing(bool);

private slots:
    void gotoCreateRoadMode(bool);
    void gotoCreateLaneMode(bool);
    void gotoDestroyMode(bool);
    void gotoModifyMode(bool);
    void gotoDragMode(bool c=true);

    void OnMouseMove(RoadRunner::MouseAction);

private:
    void SetEditMode(RoadRunner::EditMode aMode);

    RoadDrawingSession* drawingSession = nullptr;

    QButtonGroup* pointerModeGroup;
    QToolButton* createModeButton, * createLaneModeButton, * destroyModeButton, * modifyModeButton, * dragModeButton;

    MapView* mapView;

    //QToolButton* resetButton;
    SectionProfileConfigWidget* createRoadOption;

    unsigned int nRepaints = 0;
    qint64 lastUpdateFPSMS = 0;

    /*Hidden foreground scale after some time from scroll*/
    QTimer* displayScaleTimer;
};