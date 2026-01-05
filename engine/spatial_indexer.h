#pragma once

#include <QMatrix4x4>
#include <unordered_set>

#include "Road.h"
#include "id_generator.h"

namespace LM
{
    typedef uint64_t FaceIndex_t;

    struct RayCastQuery
    {
        odr::Vec3D origin;
        odr::Vec3D direction;
        std::unordered_set<FaceIndex_t> skip;
    };

    struct RayCastResult
    {
        bool hit = false;
        odr::Vec3D hitPos;
        std::string roadID;
        int lane;
        double s;
    };

    struct OverlapResult
    {
        // roadID1 is trivial as road1 is temporary
        std::string roadID2;
        int lane1, lane2;
        double sBegin1, sEnd1;
        double sBegin2, sEnd2;
    };

    // Short segment of lane, stored in spatial indexer tree
    struct Quad
    {
        std::string roadID;
        const int laneIDNormal, laneIDReversed;
        double sBegin, sEnd;
        odr::Vec3D pointS1T1, pointS2T1; // don't necessary correspond to sBegin
        odr::Vec3D pointS1T2, pointS2T2; // don't necessary correspond to sEnd
        bool magneticArea; // Quad is for mouse ray-cast query only, ignored in overlap query

        int LaneID() const
        {
            return sBegin < sEnd ? laneIDNormal : laneIDReversed;
        }

    };

    bool Overlap2D(const Quad& q1, const Quad& q2);
    bool Overlap3D(const Quad& q1, const Quad& q2, double zThreshold);

    class SpatialIndexer_impl;

    class SpatialIndexer
    {
    public:
        static SpatialIndexer* Instance();

        FaceIndex_t Index(odr::Road road, odr::Lane lane, double sBegin, double sEnd);

        RayCastResult RayCast(RayCastQuery ray);

        std::vector<OverlapResult> AllOverlaps(
            const LaneProfile& p, const odr::RefLine& l,
            double sBegin, double sEnd, double zRange = 0.01);

        void UnIndex(FaceIndex_t index);

        void RebuildTree();

        std::shared_ptr<Quad> FaceInfo(FaceIndex_t);

        void Clear();

        static uint32_t InvalidFace;

    private:
        SpatialIndexer();

        static SpatialIndexer* _instance;

        SpatialIndexer_impl* _impl;
    };

    class SpatialIndexerDynamic_impl;

    class SpatialIndexerDynamic
    {
    public:
        static SpatialIndexerDynamic* Instance();

        // New or update
        void Index(unsigned int id, QMatrix4x4 transform, QVector3D lwh);

        // returns -1 if no intersection
        unsigned int RayCast(odr::Vec3D origin, odr::Vec3D direction);

        void UnIndex(unsigned int id);
    private:
        static SpatialIndexerDynamic* _instance;

        SpatialIndexerDynamic();

        SpatialIndexerDynamic_impl* _impl;
    };
}