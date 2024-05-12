#pragma once

#include "world.h"
#include "qgraphicsview.h"
#include "qgraphicsitem.h"
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

    virtual void Complete() = 0;

    virtual ~RoadDrawingSession() {}

    void SetHighlightTo(std::shared_ptr<RoadRunner::Road>);

protected:
    float SnapDistFromScale() const;

    QGraphicsView* view;
    QGraphicsScene* scene;
    World* world;

private:
    std::weak_ptr<RoadRunner::Road> highlighted;
};

class CustomCursorItem : public QGraphicsEllipseItem
{
public:
    CustomCursorItem() : QGraphicsEllipseItem(
        -InitialRadius, -InitialRadius, 
        2 * InitialRadius, 2 * InitialRadius) {
        setZValue(99);
    }

    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget = nullptr) override;

    void EnableHighlight(bool enable);

    static double SnapRadiusPx;

private:
    static double InitialRadius;
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

private:
    std::unique_ptr<odr::RoadGeometry> createJoinAtEndGeo(bool forPreview);

    void tryCreateJunction(std::shared_ptr<RoadRunner::Road>, double);

    std::vector<QPointF> ctrlPoints;
    
    std::unique_ptr<QVector2D> startDir;
    std::weak_ptr<RoadRunner::Road> extendFromStart;
    odr::RoadLink::ContactPoint extendFromStartContact;

    std::weak_ptr<RoadRunner::Road> joinAtEnd;
    odr::RoadLink::ContactPoint joinAtEndContact;
    
    QPainterPath setPath;
    QGraphicsPathItem* setPreviewItem;

    QPainterPath flexPath;
    QGraphicsPathItem* flexPreviewItem;

    QPainterPath hintPath;
    QGraphicsPathItem* hintItem;

    CustomCursorItem* cursorItem;
};

class RoadDestroySession : public RoadDrawingSession
{
public:
    RoadDestroySession(QGraphicsView* aView);

    virtual ~RoadDestroySession() override;

    virtual bool Update(QMouseEvent* event) override;

    virtual void Complete() override;

protected:
    double GetAdjustedS();

    QGraphicsPathItem* previewItem;

    QPolygonF hintPolygon;
    QGraphicsPolygonItem* hintItem;

    CustomCursorItem* cursorItem;

    std::weak_ptr<RoadRunner::Road> targetRoad;
    std::unique_ptr<double> s1, s2;
};

class RoadModificationSession : public RoadDestroySession
{
public:
    RoadModificationSession(QGraphicsView* aView);

    //virtual ~RoadModificationSession() override;

    virtual bool Update(QMouseEvent* event) override;

    virtual void Complete() override;
};