#pragma once
#include <qgraphicsitem.h>
#include <qbrush.h>
#include <map>

#include "road.h"
#include "spatial_indexer.h"

namespace RoadRunner
{
    namespace
    {
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
        void Hide(bool hidden);

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

        unsigned int roadID;
    };

    class JunctionGraphics
    {
    public:
        JunctionGraphics(const odr::Line2D& normalBoundary, double eleation, std::string junctionID);

        JunctionGraphics(const std::vector<std::pair<odr::Line3D, odr::Line3D>>& directCavities, std::string junctionID);

        void Hide(bool hidden);

        ~JunctionGraphics();

    private:
        odr::Vec3D StripMidPoint(const odr::Vec3D& pOrigin, const odr::Vec3D& p1, const odr::Vec3D& p2);

        double junctionElevation = 0;

        const QBrush DefaultBrush = QBrush(Qt::darkGray, Qt::SolidPattern);

        std::vector<unsigned int> allGraphicsIndice;

        const unsigned int junctionObjectID; // = junctionID + offset
    };
}
