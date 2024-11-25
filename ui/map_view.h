#pragma once

#include <QGraphicsView>
#include <qevent.h>
#include <optional>

#include "road_profile.h"
#include "action_defs.h"

class MainWidget;

class RoadDrawingSession;


class MapView : public QGraphicsView
{
    Q_OBJECT
public:
    MapView(MainWidget* v, QGraphicsScene* scene);

    void SetEditMode(RoadRunner::EditMode aMode);

    void SetBackground(const QPixmap&);

    void PostEditActions();

    void ResetSceneRect();

    void OnMousePress(const RoadRunner::MouseAction&);
    void OnMouseDoubleClick(const RoadRunner::MouseAction&);
    void OnMouseMove(const RoadRunner::MouseAction&);
    void OnMouseRelease(const RoadRunner::MouseAction&);
    void OnKeyPress(const RoadRunner::KeyPressAction&);
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

    void drawForeground(QPainter* painter, const QRectF& rect) override;

private:
    const double ViewPadding = 100; // meters

    RoadDrawingSession* drawingSession = nullptr;

    RoadRunner::EditMode editMode = RoadRunner::Mode_None;

    bool showingScale = false;

    QGraphicsItem* backgroundItem = nullptr;

    QTransform lastTransform;

    std::optional<QPoint> prevDragMousePos;

    void handleException(std::exception);

    void confirmEdit();
    void quitEdit();

    void SnapCursor(const QPointF&);

    void AdjustSceneRect();
};