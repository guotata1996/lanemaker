#pragma once
#include <qgraphicsitem.h>
#include <qbrush.h>

#include "road.h"
#include "spatial_indexer.h"
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

        odr::Line3D TwoDTo3D(const odr::Line2D l, double elevation)
        {
            odr::Line3D rtn;
            rtn.resize(l.size());
            for (int i = 0; i != l.size(); ++i)
            {
                rtn[i] = odr::Vec3D{ l[i][0], l[i][1], elevation };
            }
            return rtn;
        }

        std::vector<QPolygonF> ArrowShape(int arrowType);
    }

    class LaneGraphics;

    class SectionGraphics : public QGraphicsRectItem
    {
    public:
        SectionGraphics(std::shared_ptr<RoadRunner::Road> road, const odr::LaneSection& laneSection,
            double s_begin, double s_end);

        ~SectionGraphics();

        void EnableHighlight(bool enabled, bool bringToTop=true);

        /*Assuming graphics remain the same.*/
        void updateIndexingInfo(std::string newRoadID, double sBegin, double sEnd);

        double SBegin() const;
        double SEnd() const;
        double Length() const;

        /*sBegin->sEnd follows the direction of generated LaneGraphics, NOT the direction of road ref line
          so it's possible that sBegin > sEnd*/
        //double sBegin, sEnd;
        //const double Length;
        double sectionElevation;

        std::vector<FaceIndex_t> allSpatialIndice;
        std::vector<std::size_t> allGraphicsIndice;

    private:
        static QPainterPath CreateRefLinePath(const odr::Line3D& center);

        const double BrokenLength = 3;
        const double BrokenGap = 6;

        std::vector< LaneGraphics*> allLaneGraphics;
        QGraphicsPathItem* refLineHint;

        QPainterPath refLinePath, refLinePathReversed;
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

        std::shared_ptr<Road> GetRoad() const;

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
        JunctionGraphics(const odr::Line2D& normalBoundary, double eleation);

        JunctionGraphics(const std::vector<std::pair<odr::Line2D, odr::Line2D>>& directCavities, double elevation);

        ~JunctionGraphics();

    protected:
        void hoverEnterEvent(QGraphicsSceneHoverEvent* evt) override;

        void hoverLeaveEvent(QGraphicsSceneHoverEvent* evt) override;

    private:
        odr::Vec2D StripMidPoint(const odr::Vec2D& pOrigin, const odr::Vec2D& p1, const odr::Vec2D& p2);

        double junctionElevation = 0;

        const QBrush DefaultBrush = QBrush(Qt::darkGray, Qt::SolidPattern);

        std::vector<std::size_t> allGraphicsIndice;
    };
}
