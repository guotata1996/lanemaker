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

        RoadGraphics(std::shared_ptr<RoadRunner::Road> road);

        ~RoadGraphics();

        void Update(const odr::LaneSection& section, double s_begin, double s_end);

        std::weak_ptr<RoadRunner::Road> road;

        static QPolygonF LineToPoly(const odr::Line3D& line);

    private:

        std::map<double, QGraphicsItem*> sections;
    };

    class LaneSegmentGraphics : QGraphicsPolygonItem
    {
    public:
        LaneSegmentGraphics(const QPolygonF& poly, 
            odr::Line3D outerBorder, odr::Line3D innerBorder,
            double aSBegin, double aSEnd, std::string roadID, std::string laneType,
            QGraphicsItem* parent);

        void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;

        void hoverMoveEvent(QGraphicsSceneHoverEvent* event) override;

        void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;

    protected:
        void mousePressEvent(QGraphicsSceneMouseEvent* event) override;

    private:
        std::vector<QPolygonF> subdivisionPolys;
        std::vector<double> subdivisionS;

        double sBegin, sEnd;
        std::string roadID;
    };
}



