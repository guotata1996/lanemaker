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

class GraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    GraphicsView(View* v);

protected:
#if QT_CONFIG(wheelevent)
    void wheelEvent(QWheelEvent*) override;
#endif

    void mousePressEvent(QMouseEvent* event) override;

    void mouseMoveEvent(QMouseEvent* event) override;

    void keyPressEvent(QKeyEvent* event) override;

    void AdjustSceneRect();

private:
    const double ViewPadding = 100; // meters

    View* view;
    RoadDrawingSession* drawingSession = nullptr;

signals:
    void enableRoadEditingTool(bool);

public slots:
    void confirmEdit();
    void quitEdit();
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
    void togglePointerMode();
    void toggleAntialiasing();
    void rotateLeft();
    void rotateRight();

    void showRoadEditingTool(bool show);

private:
    GraphicsView* graphicsView;
    QLabel* label2;
    QToolButton* selectModeButton;
    QToolButton* dragModeButton;
    QToolButton* antialiasButton;
    QToolButton* resetButton;
    QSlider* zoomSlider;
    QSlider* rotateSlider;

    QWidget* roadEditingTool;
};

#endif // VIEW_H