#pragma once

#include "world.h"
#include "action_defs.h"

#include "qgraphicsview.h"
#include "qgraphicsitem.h"
#include <vector>
#include <QPainterPath>

#include <map>


class RoadDrawingSession
{
public:
    RoadDrawingSession(QGraphicsView* aView);

    /*return false if force complete*/
    virtual bool Update(const RoadRunner::MouseAction&);

    /*return false to abort change*/
    virtual bool Complete() = 0;

    virtual ~RoadDrawingSession() {}

    void SetHighlightTo(std::shared_ptr<RoadRunner::Road>);

protected:
    class CustomCursorItem : public QGraphicsEllipseItem
    {
    public:
        CustomCursorItem() : QGraphicsEllipseItem(
            -InitialRadius, -InitialRadius,
            2 * InitialRadius, 2 * InitialRadius) {
            setZValue(999);
        }

        virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget = nullptr) override;

        void EnableHighlight(int level);

    private:
        static double InitialRadius;
    };

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
    bool PickProfileMode() const { return !beginPickingRoad.expired(); }

    static bool IsElevationConsistWithExtend();
    static bool IsProfileChangePoint(const std::shared_ptr<RoadRunner::Road>&, double s);

    static void UpdateEndMarkings();

    QGraphicsView* view;
    QGraphicsScene* scene;
    World* world;

    CustomCursorItem* cursorItem;

private:
    std::weak_ptr<RoadRunner::Road> highlighted;

    double beginPickingS;
    std::weak_ptr<RoadRunner::Road> beginPickingRoad;
};

class RoadCreationSession : public RoadDrawingSession
{
public:
    RoadCreationSession(QGraphicsView* aView);

    virtual bool Update(const RoadRunner::MouseAction&);

    virtual bool Complete() override;

    virtual ~RoadCreationSession();

protected:
    // Record extend / join
    virtual SnapResult SnapFirstPointToExisting(odr::Vec2D&);
    virtual SnapResult SnapLastPointToExisting(odr::Vec2D&);

    std::weak_ptr<RoadRunner::Road> extendFromStart;
    double extendFromStartS;
    std::weak_ptr<RoadRunner::Road> joinAtEnd;
    double joinAtEndS;

    // Record overlap (for junction / bridge / tunnel) // TODO:
    std::weak_ptr<RoadRunner::Road> overlapAtStart;
    double overlapAtStartS;
    std::weak_ptr<RoadRunner::Road> overlapAtEnd;
    double overlapAtEndS;

    virtual odr::Vec2D ExtendFromDir() const;
    virtual odr::Vec2D JoinAtEndDir() const;

    odr::RefLine ResultRefLine() const;

    virtual RoadRunner::type_t PreviewRightOffsetX2() const;
    virtual RoadRunner::type_t PreviewLeftOffsetX2() const;

private:
    class DirectionHandle : public QGraphicsPixmapItem
    {
    public:
        DirectionHandle();

        bool Update(const RoadRunner::MouseAction& act);

    protected:
        virtual bool contains(const QPointF& point) const override;

        virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
            QWidget* widget = nullptr) override;

    private:
        double dragging = false;
        double deltaRotation;
    };

    struct StagedGeometry
    {
        std::unique_ptr<odr::RoadGeometry> geo;
        QPainterPath refLinePreview;
        QPainterPath boundaryPreview;
    };

    SnapResult SnapCursor(odr::Vec2D&);

    std::vector<StagedGeometry> stagedGeometries;
    RoadRunner::LanePlan stagedLeftPlan, stagedRightPlan;

    std::optional<odr::Vec2D> startPos; // Can be on blank or extend from

    void GeneratePainterPath(const std::unique_ptr<odr::RoadGeometry>&,
        QPainterPath&, QPainterPath&);

    void UpdateStagedFromGeometries(bool lanePlanChanged = false);

    std::unique_ptr<odr::RoadGeometry> flexGeo;
    QPainterPath flexRefLinePath;
    QPainterPath flexBoundaryPath;
    QPainterPath stagedRefLinePath;
    QPainterPath stagedBoundaryPath;

    QGraphicsPathItem* stagedRefLinePreview;
    QGraphicsPathItem* stagedBoundaryPreview;
    QGraphicsPathItem* flexRefLinePreview;
    QGraphicsPathItem* flexBoundaryPreview;
    DirectionHandle* directionHandle;
};


class LanesCreationSession : public RoadCreationSession
{
public:
    LanesCreationSession(QGraphicsView* aView);

    virtual bool Complete() override;

    virtual ~LanesCreationSession() override;

protected:
    virtual SnapResult SnapFirstPointToExisting(odr::Vec2D&) override;
    virtual SnapResult SnapLastPointToExisting(odr::Vec2D&) override;
    virtual odr::Vec2D ExtendFromDir() const override;
    virtual odr::Vec2D JoinAtEndDir() const override;

    virtual RoadRunner::type_t PreviewRightOffsetX2() const override;
    virtual RoadRunner::type_t PreviewLeftOffsetX2() const override;

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

    virtual bool Update(const RoadRunner::MouseAction&) override;

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

    virtual bool Update(const RoadRunner::MouseAction&) override;

    virtual bool Complete() override;
};