#pragma once

#include "world.h"
#include "qgraphicsview.h"
#include <vector>
#include <QPainterPath>

class RoadDrawingSession
{
public:
    RoadDrawingSession(QGraphicsView* aView);
    
    /*return false if force complete*/
    bool Update(QMouseEvent* event);

    void Complete();

    ~RoadDrawingSession();

    void SnapCtrlPoint(float maxOffset);

    void CreateRoad();

    bool IsRoadValid() { return ctrlPoints.size() > 3 || ctrlPoints.size() == 3 && !joinAtEnd.expired(); }

private:
    QGraphicsView* view;
    QGraphicsScene* scene;

    std::vector<QPointF> ctrlPoints;
    
    std::unique_ptr<QVector2D> startDir;
    std::weak_ptr<RoadRunner::Road> extendFromStart;
    odr::RoadLink::ContactPoint extendFromStartContact;

    std::weak_ptr<RoadRunner::Road> joinAtEnd;
    odr::RoadLink::ContactPoint joinAtEndContact;
    
    QPainterPath ctrlPath;
    QGraphicsPathItem* previewItem;

    QPainterPath hintPath;
    QGraphicsPathItem* hintItem;

    QGraphicsEllipseItem* cursorItem;

    const double SnapToEndThreshold = 3;

    World* world;
    
};