#pragma once
#include <QFrame>
#include <QGraphicsView>

QT_BEGIN_NAMESPACE
class QLabel;
class QSlider;
class QToolButton;
class QButtonGroup;
QT_END_NAMESPACE

class MapView;

class SectionProfileConfigWidget;

class MainWidget : public QFrame
{
    Q_OBJECT
public:
    explicit MainWidget(QGraphicsScene* scene, QWidget* parent = nullptr);

    QGraphicsView* view() const;

    void AdjustSceneRect();

    void SetHovering(QString);

    void Painted();

    void Reset();

    void RecordViewTransform();
    void SetViewFromReplay(double zoomSliderVal, double rotateSliderVal);
    void SetModeFromReplay(int mode);

signals:
    void HoveringChanged(QString);

    void FPSChanged(QString);

    void InReadOnlyMode(bool);

public slots:
    void zoomIn();
    void zoomOut();
    void zoomInBy(int level);
    void zoomOutBy(int level);
    void toggleAntialiasing(bool);

private slots:
    void resetView();
    void setResetButtonEnabled();
    void setupMatrix();

    void gotoCreateRoadMode();
    void gotoCreateLaneMode();
    void gotoDestroyMode();
    void gotoModifyMode();
    void gotoDragMode();
    void rotateLeft();
    void rotateRight();

private:
    QButtonGroup* pointerModeGroup;
    QToolButton* createModeButton, * createLaneModeButton, * destroyModeButton, * modifyModeButton, * dragModeButton;

    MapView* mapView;
    QToolButton* resetButton;
    QSlider* zoomSlider = nullptr;
    QSlider* rotateSlider = nullptr;

    SectionProfileConfigWidget* createRoadOption;

    unsigned int nRepaints = 0;
    qint64 lastUpdateFPSMS = 0;

    /*Hidden foreground scale after some time from scroll*/
    QTimer* displayScaleTimer;
};