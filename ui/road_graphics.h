#pragma once
#include <qgraphicsitem.h>
#include <qbrush.h>
#include <map>

#include "road.h"
#include "spatial_indexer.h"

namespace LM
{
    namespace
    {
        std::vector<QPolygonF> ArrowShape(int arrowType);
    }

    // TODO: g_mapViewGL->AddXXX is Only accessable through AbstractGraphicsItem
    class AbstractGraphicsItem
    {
    public:
        virtual void AddQuads(const odr::Line3D& lBorder, const odr::Line3D& rBorder, QColor color);
        virtual void AddLine(const odr::Line3D& border, double width, QColor color);
        virtual void AddPoly(const odr::Line3D& boundary, QColor color, double h = 0);

        void Clear();

        ~AbstractGraphicsItem();

    protected:
        AbstractGraphicsItem() = default;

        std::vector<int> graphicsIndex;

        unsigned int objectID;
    };

    class TemporaryGraphics: public AbstractGraphicsItem
    {
    public:
        TemporaryGraphics();
    };

    class PermanentGraphics: public AbstractGraphicsItem
    {
    public:
        PermanentGraphics(unsigned int objectID);

        void UpdateObjectID(unsigned int objectID);
        void RemoveObject();
        void UpdateObject(uint8_t);
    };

    class HintLineGraphics: protected TemporaryGraphics
    {
    public:
        HintLineGraphics(const odr::Line3D& boundaryL, const odr::Line3D& boundaryR, QColor color);
        HintLineGraphics(const odr::Line3D& center, double width, QColor color);
    };

    class HintPolyGraphics : protected TemporaryGraphics
    {
    public:
        HintPolyGraphics(const odr::Line3D& boundary, QColor color, double height = 0);
    };

    class SectionGraphics: protected PermanentGraphics
    {
    public:
        SectionGraphics(std::shared_ptr<LM::Road> road, const odr::LaneSection& laneSection,
            double s_begin, double s_end);

        ~SectionGraphics();

        /*Assuming graphics remain the same.*/
        void updateIndexingInfo(std::string newRoadID, int mult, double shift);

        double Length() const;

        double sMin, sMax;
        
        std::vector<FaceIndex_t> allSpatialIndice;

    private:
        static QPainterPath CreateRefLinePath(const odr::Line3D& center);

        const double BrokenLength = 3;
        const double BrokenGap = 6;
    };

    class JunctionGraphics: protected PermanentGraphics
    {
    public:
        JunctionGraphics(const odr::Line2D& normalBoundary, double eleation, std::string junctionID);

        JunctionGraphics(const std::vector<std::pair<odr::Line3D, odr::Line3D>>& directCavities, std::string junctionID);

        void Hide(bool hidden);

        ~JunctionGraphics();

    private:
        odr::Vec3D StripMidPoint(const odr::Vec3D& pOrigin, const odr::Vec3D& p1, const odr::Vec3D& p2);
    };

    struct InstanceData
    {
        unsigned int variation;
        QColor color;

        static InstanceData GetRandom();
    };

    class InstancedGraphics
    {
    public:
        InstancedGraphics(unsigned int objectID, InstanceData);

        ~InstancedGraphics();

        void SetTransform(QMatrix4x4 trans);
    private:
        unsigned int objectID;
        unsigned int variation;
    };
}
