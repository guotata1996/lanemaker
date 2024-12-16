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

    class TemporaryGraphics
    {
    public:
        TemporaryGraphics(const odr::Line3D& boundaryL, const odr::Line3D& boundaryR, QColor color);
        TemporaryGraphics(const odr::Line3D& center, double width, QColor color);

        ~TemporaryGraphics();
    private:
        unsigned int graphicsIndex;
    };

    class SectionGraphics
    {
    public:
        SectionGraphics(std::shared_ptr<RoadRunner::Road> road, const odr::LaneSection& laneSection,
            double s_begin, double s_end);

        ~SectionGraphics();

        void EnableHighlight(bool enabled);

        /*Assuming graphics remain the same.*/
        void updateIndexingInfo(std::string newRoadID, int mult, double shift);

        double Length() const;

        double sMin, sMax;
        
        double sectionElevation;

        std::vector<FaceIndex_t> allSpatialIndice;
        std::vector<unsigned int> allGraphicsIndice;
        std::vector<unsigned int> allHighlightGraphicsIndice;

    private:
        static QPainterPath CreateRefLinePath(const odr::Line3D& center);

        const double BrokenLength = 3;
        const double BrokenGap = 6;
    };

    class JunctionGraphics
    {
    public:
        JunctionGraphics(const odr::Line2D& normalBoundary, double eleation);

        JunctionGraphics(const std::vector<std::pair<odr::Line2D, odr::Line2D>>& directCavities, double elevation);

        ~JunctionGraphics();

    protected:
        //void hoverEnterEvent(QGraphicsSceneHoverEvent* evt) override;

        //void hoverLeaveEvent(QGraphicsSceneHoverEvent* evt) override;

    private:
        odr::Vec2D StripMidPoint(const odr::Vec2D& pOrigin, const odr::Vec2D& p1, const odr::Vec2D& p2);

        double junctionElevation = 0;

        const QBrush DefaultBrush = QBrush(Qt::darkGray, Qt::SolidPattern);

        std::vector<unsigned int> allGraphicsIndice;
    };
}
