#pragma once
#include <QFrame>
#include <QGraphicsView>

QT_BEGIN_NAMESPACE
class QLabel;
class QSlider;
class QToolButton;
QT_END_NAMESPACE

class MapView;

class CreateRoadOptionWidget;

class MainWidget : public QFrame
{
    Q_OBJECT
public:
    explicit MainWidget(const QString& name, QWidget* parent = nullptr);

    QGraphicsView* view() const;

    void AdjustSceneRect();

    void SetHovering(QString);

signals:
    void HoveringChanged(QString);

public slots:
    void zoomIn();
    void zoomOut();
    void zoomInBy(int level);
    void zoomOutBy(int level);

private slots:
    void resetView();
    void setResetButtonEnabled();
    void setupMatrix();

    void gotoCreateMode();
    void gotoDestroyMode();
    void gotoDragMode();
    void toggleAntialiasing();
    void rotateLeft();
    void rotateRight();

private:
    MapView* graphicsView;
    QLabel* label2;
    QToolButton* createModeButton;
    QToolButton* destroyModeButton;
    QToolButton* dragModeButton;
    QToolButton* antialiasButton;
    QToolButton* resetButton;
    QSlider* zoomSlider;
    QSlider* rotateSlider;

    CreateRoadOptionWidget* createRoadOption;
};