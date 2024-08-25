#pragma once

#include "world.h"
#include "action_defs.h"

#include "qgraphicsview.h"
#include "qgraphicsitem.h"
#include <vector>
#include <QPainterPath>


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
    struct StagedGeometry
    {
        std::unique_ptr<odr::RoadGeometry> geo;
        QPainterPath preview;
    };
    std::vector<StagedGeometry> stagedGeometries;

    std::optional<odr::Vec2D> startPos; // Can be on blank or extend from

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

    SnapResult SnapCursor(odr::Vec2D&);

    static void GeneratePainterPath(const std::unique_ptr<odr::RoadGeometry>&,
        QPainterPath&);

    std::unique_ptr<odr::RoadGeometry> flexGeo;
    QPainterPath flexPreviewPath;
    QPainterPath stagedPreviewPath;

    QGraphicsPathItem* stagedPreview;
    QGraphicsPathItem* flexPreview;
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