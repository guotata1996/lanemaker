#pragma once

#include "world.h"
#include "qgraphicsview.h"
#include <vector>
#include <QPainterPath>

class RoadDrawingSession
{
public:
    RoadDrawingSession(QGraphicsView* aView):
        view(aView), scene(aView->scene()), world(World::Instance())
    {}

    /*return false if force complete*/
    virtual bool Update(QMouseEvent* event) = 0;

    virtual bool IsValid() = 0;

    virtual void Complete() = 0;

    virtual ~RoadDrawingSession() {}

protected:
    float ScaleFromView() const;

    float SnapDistFromScale() const;

    QGraphicsView* view;
    QGraphicsScene* scene;
    World* world;
private:
    const double SnapToEndThreshold = 10;
};

class RoadCreationSession: public RoadDrawingSession
{
public:
    RoadCreationSession(QGraphicsView* aView);
    
    /*return false if force complete*/
    virtual bool Update(QMouseEvent* event) override;

    virtual void Complete() override;

    virtual ~RoadCreationSession() override;

    bool SnapCtrlPoint(float maxOffset);

    void CreateRoad();

    virtual bool IsValid() override 
    { 
        return ctrlPoints.size() > 3 || ctrlPoints.size() == 3 && !joinAtEnd.expired(); 
    }

private:

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
};

class RoadDestroySession : public RoadDrawingSession
{
public:
    RoadDestroySession(QGraphicsView* aView);

    virtual ~RoadDestroySession() override;

    virtual bool Update(QMouseEvent* event) override;

    virtual bool IsValid() override
    {
        return false;
    }

    virtual void Complete() override;

private:
    double GetAdjustedS();

    QGraphicsPathItem* previewItem;

    QPolygonF hintPolygon;
    QGraphicsPolygonItem* hintItem;

    QGraphicsEllipseItem* cursorItem;

    std::weak_ptr<RoadRunner::Road> targetRoad;
    std::unique_ptr<double> s1, s2;
    std::set<RoadRunner::type_s> targetSectionKeys;
};