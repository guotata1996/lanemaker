#pragma once
#include <qgraphicsitem.h>

#include "road.h"
#include <map>

namespace RoadRunner
{
    namespace
    {
        QPolygonF LineToPoly(const odr::Line3D& line)
        {
            QPolygonF rtn;
            for (const odr::Vec3D& p : line)
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

        void EnableHighlight(bool enabled);

        /*Depends on ref line direction. Only this part needs updating upon reverse.*/
        void UpdateRefLineHint();

        std::weak_ptr<RoadRunner::Road> road;

        /*sBegin->sEnd follows the direction of generated LaneGraphics, NOT the direction of road ref line
          so it's possible that sBegin > sEnd*/
        double sBegin, sEnd;
        const double Length;

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
            int laneID, std::string laneType,
            QGraphicsItem* parent);

        std::weak_ptr<Road> SnapCursor(QPointF p, double& outS);

        std::shared_ptr<Road> Road() const;

        const int LaneID;

        void EnableHighlight(bool enabled);

    private:
        std::vector<QPolygonF> subdivisionPolys;
        /*0, 0.05, ..., 0.95, 1*/
        std::vector<double> subdivisionPortion;

        const QColor NormalColor;
        const QColor HighlightColor;
        bool isMedian;
    };
}
