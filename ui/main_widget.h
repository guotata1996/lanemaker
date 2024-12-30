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

    static MainWidget* Instance();

    void Painted();

    void Reset();

    void SetModeFromReplay(int mode);
    void SetElevationFromReplay(int8_t elevationSetting);

    RoadRunner::MapViewGL* mapViewGL;

signals:
    void HoveringChanged(QString);

    void FPSChanged(QString);

    void InReadOnlyMode(bool);

public slots:
    void toggleAntialiasing(bool);
    void OnMouseAction(RoadRunner::MouseAction);
    void OnKeyPress(RoadRunner::KeyPressAction);

private slots:
    void gotoCreateRoadMode(bool);
    void gotoCreateLaneMode(bool);
    void gotoDestroyMode(bool);
    void gotoModifyMode(bool);
    void gotoDragMode(bool c=true);

private:
    static MainWidget* instance;

    void SetEditMode(RoadRunner::EditMode aMode);

    void confirmEdit();
    void quitEdit();

    void elegantlyHandleException(std::exception);

    RoadRunner::EditMode editMode = RoadRunner::Mode_None;
    RoadDrawingSession* drawingSession = nullptr;

    QButtonGroup* pointerModeGroup;
    QToolButton* createModeButton, * createLaneModeButton, * destroyModeButton, * modifyModeButton, * dragModeButton;

    //QToolButton* resetButton;
    SectionProfileConfigWidget* createRoadOption;

    unsigned int nRepaints = 0;
    qint64 lastUpdateFPSMS = 0;
};