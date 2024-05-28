#pragma once
#include <qgraphicsitem.h>

#include "road.h"
#include <map>

namespace RoadRunner
{
    class LaneSegmentGraphics;

    class RoadGraphics : public QGraphicsRectItem
    {
    public:
        RoadGraphics(std::shared_ptr<RoadRunner::Road> road, const odr::LaneSection& laneSection,
            double s_begin, double s_end);

        ~RoadGraphics();

        void EnableHighlight(bool enabled);

        std::weak_ptr<RoadRunner::Road> road;

        static QPolygonF LineToPoly(const odr::Line3D& line);

        /*sBegin->sEnd follows the direction of generated LaneSegmentGraphics, NOT the direction of road ref line
          so it's possible that sBegin > sEnd*/
        double sBegin, sEnd;
        const double Length;

    private:
        void Create(const odr::LaneSection&);

        const double BrokenLength = 3;
        const double BrokenGap = 6;

        std::vector< LaneSegmentGraphics*> allSegmentGraphics;
    };

    class LaneSegmentGraphics : public QGraphicsPolygonItem
    {
        friend RoadGraphics;
    public:
        LaneSegmentGraphics(const QPolygonF& poly, 
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



