#pragma once

#include "world.h"
#include "qgraphicsview.h"
#include "qgraphicsitem.h"
#include <vector>
#include <QPainterPath>

class CustomCursorItem : public QGraphicsEllipseItem
{
public:
    CustomCursorItem() : QGraphicsEllipseItem(
        -InitialRadius, -InitialRadius,
        2 * InitialRadius, 2 * InitialRadius) {
        setZValue(99);
    }

    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget = nullptr) override;

    void EnableHighlight(int level);

    static double SnapRadiusPx;

private:
    static double InitialRadius;
};

class RoadDrawingSession
{
public:
    RoadDrawingSession(QGraphicsView* aView);

    /*return false if force complete*/
    virtual bool Update(QMouseEvent* event) = 0;

    /*return false to abort change*/
    virtual bool Complete() = 0;

    virtual ~RoadDrawingSession() {}

    void SetHighlightTo(std::shared_ptr<RoadRunner::Road>);

protected:
    enum SnapResult
    {
        Snap_Nothing,
        Snap_Line,
        Snap_Point
    };

    float SnapDistFromScale() const;

    double GetAdjustedS(bool* onSegmentBoundary = nullptr) const;

    void BeginPickingProfile();
    void ContinuePickingProfile();
    void EndPickingProfile();

    QGraphicsView* view;
    QGraphicsScene* scene;
    World* world;

    CustomCursorItem* cursorItem;

    bool PickProfileMode() const { return !beginPickingRoad.expired(); }

private:
    std::weak_ptr<RoadRunner::Road> highlighted;

    double beginPickingS;
    std::weak_ptr<RoadRunner::Road> beginPickingRoad;
};

class RoadCreationSession: public RoadDrawingSession
{
public:
    RoadCreationSession(QGraphicsView* aView);
    
    /*return false if force complete*/
    virtual bool Update(QMouseEvent* event) override;

    virtual bool Complete() override;

    virtual ~RoadCreationSession() override;

protected:
    virtual SnapResult SnapFirstPointToExisting(QPointF&);
    virtual SnapResult SnapLastPointToExisting(QPointF&);

    SnapResult SnapCtrlPoint(float maxOffset);

    odr::RefLine RefLineFromCtrlPoints() const;

    enum class ForceDirection
    {
        None, Original, Negate
    };

    virtual std::unique_ptr<odr::RoadGeometry> CreateJoinAtEndGeo(bool forPreview) const;

    virtual bool CreateRoad();

    std::unique_ptr<odr::RoadGeometry> createJoinAtEndGeo(bool forPreview, ForceDirection joinPointDir) const;

    std::vector<QPointF> ctrlPoints;

    std::unique_ptr<QVector2D> startDir;
    std::weak_ptr<RoadRunner::Road> extendFromStart;
    std::weak_ptr<RoadRunner::Road> joinAtEnd;

    double extendFromStartS, joinAtEndS;

    bool lastClickOnExtLine; /*Prohibit random dbl-click*/

private:
    void tryCreateJunction(std::shared_ptr<RoadRunner::Road>, double, double);
    
    QPainterPath setPath;
    QGraphicsPathItem* setPreviewItem;

    QPainterPath flexPath;
    QGraphicsPathItem* flexPreviewItem;

    QPainterPath hintPath;
    QGraphicsPathItem* hintItem;
};


class LanesCreationSession : public RoadCreationSession
{
public:
    LanesCreationSession(QGraphicsView* aView);

    virtual bool CreateRoad() override;

    virtual ~LanesCreationSession() override;

protected:
    virtual SnapResult SnapFirstPointToExisting(QPointF&) override;
    virtual SnapResult SnapLastPointToExisting(QPointF&) override;

    virtual std::unique_ptr<odr::RoadGeometry> CreateJoinAtEndGeo(bool forPreview) const override;

private:
    bool ValidateSnap() const;

    RoadRunner::type_t rLanes; // If lLanes == 0, becomes required lanes
    RoadRunner::type_t lLanes; // If 0, single-directional
    RoadRunner::type_t rOffsetX2, lOffsetX2;

    // Offset of ramp w.r.t interface provider
    // Only != 0 when single-direction ramp
    uint8_t startLanesSkip, endLanesSkip;

    // Offset of narrower (not interface-provider) half of split result w.r.t. interface provider
    // Only != 0 at left-exit from single-directional origin
    uint8_t startSplitOffset, endSplitOffset;

    int startSide, endSide; // Only != 0 when single-direction ramp

    bool startFullyMatch, endFullyMatch; // If true, extend instead of creating direct junction
};

class RoadDestroySession : public RoadDrawingSession
{
public:
    RoadDestroySession(QGraphicsView* aView);

    virtual ~RoadDestroySession() override;

    virtual bool Update(QMouseEvent* event) override;

    virtual bool Complete() override;

protected:
    QGraphicsPathItem* previewItem;

    QPolygonF  hintPolygonLeft, hintPolygonRight;
    QGraphicsPolygonItem* hintItemLeft, *hintItemRight;

    std::weak_ptr<RoadRunner::Road> targetRoad;
    std::unique_ptr<double> s1, s2;
};

class RoadModificationSession : public RoadDestroySession
{
public:
    RoadModificationSession(QGraphicsView* aView);

    //virtual ~RoadModificationSession() override;

    virtual bool Update(QMouseEvent* event) override;

    virtual bool Complete() override;
};