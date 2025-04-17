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
namespace LM
{
    class MapViewGL;
}

class SectionProfileConfigWidget;

class MainWidget : public QFrame
{
    Q_OBJECT
public:
    explicit MainWidget(QWidget* parent = nullptr);

    static MainWidget* Instance();

    void Painted();

    void Reset();

    void SetModeFromReplay(int mode);

    void GoToSimulationMode(bool enabled); // force into drag mode

    LM::EditMode GetEditMode() const;

    LM::MapViewGL* mapViewGL;

signals:
    void HoveringChanged(QString);

    void FPSChanged(QString);

public slots:
    void toggleAntialiasing(bool);
    void OnMouseAction(LM::MouseAction);
    void OnKeyPress(LM::KeyPressAction);

private slots:
    void gotoCreateRoadMode(bool);
    void gotoCreateLaneMode(bool);
    void gotoDestroyMode(bool);
    void gotoModifyMode(bool);
    void gotoDragMode(bool c=true);

private:
    static MainWidget* instance;

    void SetEditMode(LM::EditMode aMode);

    void confirmEdit();
    void quitEdit();

    void elegantlyHandleException(std::exception);

    LM::EditMode editMode = LM::Mode_None;
    RoadDrawingSession* drawingSession = nullptr;

    QButtonGroup* pointerModeGroup;
    QToolButton* createModeButton, * createLaneModeButton, * destroyModeButton, * modifyModeButton, * dragModeButton;

    SectionProfileConfigWidget* createRoadOption;

    unsigned int nRepaints = 0;
    qint64 lastUpdateFPSMS = 0;
};