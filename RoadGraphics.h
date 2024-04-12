#pragma once
#include <qgraphicsitem.h>

#include "road.h"
#include <map>

namespace RoadRunner
{
    class RoadGraphics : public QGraphicsRectItem
    {
    public:
        // stores {(s_range_min, s_range_b): graphicsItem of LaneSection->children of lanes?}
        // s_range has max length of _ M
        // When r2 joins r1, r2's buffers get moved to r1 (except for the connecting part)
        // When r2 splits from r1, r1's buffers get moved to r2
        // when r reverses, only dict keys need update. No point calc is needed
        // When profile changes, only changed part need to re-compute
        // Newly-created r has a length limit to save performance

        RoadGraphics(std::shared_ptr<RoadRunner::Road> road, const odr::LaneSection& laneSection,
            double s_begin, double s_end);

        ~RoadGraphics();

        std::weak_ptr<RoadRunner::Road> road;

        static QPolygonF LineToPoly(const odr::Line3D& line);

        /*sBegin->sEnd follows the direction of generated LaneSegmentGraphics,
        NOT the direction of road ref line, i.e. sBegin can > sEnd*/
        double sBegin, sEnd;
        const double Length;

    private:
        void Create(const odr::LaneSection&);

        const double BrokenLength = 3;
        const double BrokenGap = 6;
    };

    class LaneSegmentGraphics : public QGraphicsPolygonItem
    {
        friend RoadGraphics;
    public:
        LaneSegmentGraphics(const QPolygonF& poly, 
            odr::Line3D outerBorder, odr::Line3D innerBorder,
            std::string laneType,
            QGraphicsItem* parent);

        bool SnapCursor(QPointF p);

        std::shared_ptr<Road> Road() const;

    protected:
        void mousePressEvent(QGraphicsSceneMouseEvent* event) override;

    private:
        std::vector<QPolygonF> subdivisionPolys;
        /*0, 0.05, ..., 0.95, 1*/
        std::vector<double> subdivisionPortion;
    };
}



