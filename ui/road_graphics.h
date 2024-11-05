#pragma once
#include <qgraphicsitem.h>
#include <qbrush.h>

#include "road.h"
#include <map>

namespace RoadRunner
{
    namespace
    {
        template <class T>
        QPolygonF LineToPoly(const std::vector<T>& line)
        {
            QPolygonF rtn;
            for (const T& p : line)
            {
                rtn.append(QPointF(p[0], p[1]));
            }
            return rtn;
        }
    }

    class LaneGraphics;

    class SectionGraphics : public QGraphicsRectItem
    {
    public:
        SectionGraphics(std::shared_ptr<RoadRunner::Road> road, const odr::LaneSection& laneSection,
            double s_begin, double s_end);

        ~SectionGraphics();

        void EnableHighlight(bool enabled, bool bringToTop=true);

        /*Depends on ref line direction. Only this part needs updating upon reverse.*/
        void UpdateRefLineHint();

        std::weak_ptr<RoadRunner::Road> road;

        /*sBegin->sEnd follows the direction of generated LaneGraphics, NOT the direction of road ref line
          so it's possible that sBegin > sEnd*/
        double sBegin, sEnd;
        const double Length;
        double sectionElevation;

    private:
        void Create(const odr::LaneSection&);

        const double BrokenLength = 3;
        const double BrokenGap = 6;

        std::vector< LaneGraphics*> allLaneGraphics;
        QGraphicsPathItem* refLineHint;
    };

    class LaneGraphics : public QGraphicsPolygonItem
    {
        friend SectionGraphics;
    public:
        LaneGraphics(const QPolygonF& poly, 
            odr::Line3D outerBorder, odr::Line3D innerBorder,
            int laneID, int laneIDRev, std::string laneType,
            QGraphicsItem* parent);

        void paint(QPainter* painter, const QStyleOptionGraphicsItem* item, QWidget* widget) override;

        std::weak_ptr<Road> SnapCursor(QPointF p, double& outS);

        std::shared_ptr<Road> GetRoad() const;

        int LaneID() const;

        void EnableHighlight(bool enabled);

    private:
        QPolygonF lowLODPoly;

        std::vector<QPolygonF> subdivisionPolys;
        /*0, 0.05, ..., 0.95, 1*/
        std::vector<double> subdivisionPortion;

        const QColor HighlightColor;
        bool isMedian;

        const int laneID, laneIDReversed;
    };

    class MarkingGraphics : public QGraphicsPolygonItem
    {
    public:
        MarkingGraphics(const QPolygonF& polygon, QGraphicsItem* parent = nullptr);

        void paint(QPainter* painter, const QStyleOptionGraphicsItem* item, QWidget* widget) override;
    private:
        QPolygonF lowLODPoly;
    };

    class JunctionGraphics : public QGraphicsPathItem
    {
    public:
        JunctionGraphics(const odr::Line2D& normalBoundary);

        JunctionGraphics(const std::vector<std::pair<odr::Line2D, odr::Line2D>>& directCavities);

        ~JunctionGraphics();

    protected:
        void hoverEnterEvent(QGraphicsSceneHoverEvent* evt) override;

        void hoverLeaveEvent(QGraphicsSceneHoverEvent* evt) override;

    private:
        odr::Vec2D StripMidPoint(const odr::Vec2D& pOrigin, const odr::Vec2D& p1, const odr::Vec2D& p2);

        double junctionElevation = 0;

        const QBrush DefaultBrush = QBrush(Qt::darkGray, Qt::SolidPattern);
    };

    class ArrowGraphics : public QGraphicsPathItem
    {
    public:
        ArrowGraphics(int arrowType, QGraphicsItem* parent);
    };
}
