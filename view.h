// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef VIEW_H
#define VIEW_H

#include <QFrame>
#include <qmenu.h>
#include <QGraphicsView>

QT_BEGIN_NAMESPACE
class QLabel;
class QSlider;
class QToolButton;
QT_END_NAMESPACE

class View;

class RoadDrawingSession;

class CreateRoadOptionWidget;

class GraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    GraphicsView(View* v);

    enum EditMode
    {
        Mode_None,
        Mode_Create,
        Mode_Destroy
    };

    void SetEditMode(EditMode aMode);

protected:
#if QT_CONFIG(wheelevent)
    void wheelEvent(QWheelEvent*) override;
#endif

    void mousePressEvent(QMouseEvent* event) override;

    void mouseMoveEvent(QMouseEvent* event) override;

    void keyPressEvent(QKeyEvent* event) override;

    void drawForeground(QPainter* painter, const QRectF& rect) override;

    void AdjustSceneRect();

private:
    const double ViewPadding = 100; // meters

    View* view;
    RoadDrawingSession* drawingSession = nullptr;

    EditMode editMode = Mode_None;

    void confirmEdit();
    void quitEdit();

    void SnapCursor(const QPoint&);
};

class View : public QFrame
{
    Q_OBJECT
public:
    explicit View(const QString& name, QWidget* parent = nullptr);

    QGraphicsView* view() const;

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
    GraphicsView* graphicsView;
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

#endif // VIEW_H