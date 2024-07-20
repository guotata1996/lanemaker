#pragma once

#include <QGraphicsView>
#include <qevent.h>

#include "road_profile.h"

class MainWidget;

class RoadDrawingSession;


class MapView : public QGraphicsView
{
    Q_OBJECT
public:
    MapView(MainWidget* v);

    enum EditMode
    {
        Mode_None,
        Mode_Create,
        Mode_CreateLanes,
        Mode_Destroy,
        Mode_Modify
    };

    void SetEditMode(EditMode aMode);

    void AdjustSceneRect();

    void ResetSceneRect();

    void OnMousePress(QMouseEvent* event);
    void OnMouseDoubleClick(QMouseEvent* event);
    void OnMouseMove(QMouseEvent* event);
    void OnMouseRelease(QMouseEvent* event);
    void OnKeyPress(QKeyEvent* event);
    double Zoom() const;
    void SetViewFromReplay(double zoomSliderVal, double rotateSliderVal,
        int hScrollbar, int vScrollbar);

    MainWidget* parentContainer;

public slots:
    void showScale();
    void hideScale();

protected:
#if QT_CONFIG(wheelevent)
    void wheelEvent(QWheelEvent*) override;
#endif
    void scrollContentsBy(int dx, int dy) override;

    void mousePressEvent(QMouseEvent* event) override;

    void mouseDoubleClickEvent(QMouseEvent* event) override;

    void mouseMoveEvent(QMouseEvent* event) override;

    void mouseReleaseEvent(QMouseEvent* event) override;

    void keyPressEvent(QKeyEvent* event) override;

    void paintEvent(QPaintEvent*) override;

    void drawForeground(QPainter* painter, const QRectF& rect);

private:
    const double ViewPadding = 100; // meters

    RoadDrawingSession* drawingSession = nullptr;

    EditMode editMode = Mode_None;

    bool showingScale = false;

    QTransform lastTransform;

    void handleException(std::exception);

    void confirmEdit();
    void quitEdit();

    void SnapCursor(const QPoint&);
};