#pragma once

#include "world.h"
#include "action_defs.h"
#include "road_graphics.h"

#include "qgraphicsview.h"
#include "qgraphicsitem.h"
#include "Transform3D.h"
#include <QPainterPath>
#include <vector>
#include <map>


class RoadDrawingSession
{
public:
    RoadDrawingSession();

    /*return false if force complete*/
    virtual bool Update(const RoadRunner::MouseAction&);

    /*return false to abort change*/
    virtual bool Complete() = 0;

    virtual ~RoadDrawingSession();

    void SetHighlightTo(std::shared_ptr<RoadRunner::Road>);

protected:
    class CustomCursorItem
    {
    public:
        CustomCursorItem();
        ~CustomCursorItem();
        //virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget = nullptr) override;

        void EnableHighlight(int level);
        void SetTranslation(odr::Vec3D);

    private:
        odr::Vec3D transform;

        static double InitialRadius;

        std::optional<unsigned int> graphicsIndex;
        QColor color;
    };

    enum SnapResult
    {
        Snap_Nothing,
        Snap_Line,
        Snap_Point
    };

    static std::shared_ptr<RoadRunner::Road> GetPointerRoad();

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
    World* world;

    std::unique_ptr<CustomCursorItem> cursorItem;

private:
    std::weak_ptr<RoadRunner::Road> highlighted;

    double beginPickingS;
    std::weak_ptr<RoadRunner::Road> beginPickingRoad;
};

class RoadCreationSession : public RoadDrawingSession
{
public:
    RoadCreationSession();

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
        odr::Line3D refLinePreview;
        odr::Line3D boundaryPreviewL, boundaryPreviewR;
    };

    SnapResult SnapCursor(odr::Vec2D&);

    std::vector<StagedGeometry> stagedGeometries;
    RoadRunner::LanePlan stagedLeftPlan, stagedRightPlan;

    std::optional<odr::Vec2D> startPos; // Can be on blank or extend from

    void GenerateHintLines(const std::unique_ptr<odr::RoadGeometry>&,
        odr::Line3D&, odr::Line3D&, odr::Line3D&);

    void UpdateStagedFromGeometries(bool lanePlanChanged = false);

    std::unique_ptr<odr::RoadGeometry> flexGeo;
    odr::Line3D flexRefLinePath;
    odr::Line3D flexBoundaryPathL, flexBoundaryPathR;
    odr::Line3D stagedRefLinePath;
    odr::Line3D stagedBoundaryPathL, stagedBoundaryPathR;

    DirectionHandle* directionHandle; // TODO

    std::optional<RoadRunner::TemporaryGraphics> stagedRefLinePreview;
    std::optional<RoadRunner::TemporaryGraphics> stagedBoundaryPreview;
    std::optional<RoadRunner::TemporaryGraphics> flexRefLinePreview;
    std::optional<RoadRunner::TemporaryGraphics> flexBoundaryPreview;
};


class LanesCreationSession : public RoadCreationSession
{
public:
    LanesCreationSession();

    virtual bool Complete() override;

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
    RoadDestroySession();

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
    RoadModificationSession();

    virtual bool Update(const RoadRunner::MouseAction&) override;

    virtual bool Complete() override;
};