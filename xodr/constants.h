namespace RoadRunner
{
    /*Protect bad (way too long) geometry from freezing UI*/
    const double SingleDrawMaxLength = 500;

    /*Road will be rejected if ref line turns too sharp*/
    const double RoadMaxCurvature = 1.0 / 4;

    /*Consecutive ctrl points can either duplicate (by dbl-clicking), or placed further than this threshold.*/
    const double DupCtrlPointsDist = 0.1;

    const double SnapRadiusPx = 3;        
    
    /*Space for connecting road curvature*/
    const double JunctionTrimMax = 6;
    const double JunctionTrimMin = 2;

    // Determines resolution for collision detection
    const double GraphicsDivision = 5;

    // allowed gap between roads, in meters
    const double epsilon = 1e-2;

    // Elevation change of each adjustment step
    const double ElevationStep = 2.0;
    // Min clearance for creating bridge/tunnel (flat junction otherwise)
    const double BridgeClearance = 5.0;

    // Global buffer size
    const uint32_t MaxObjectID = 16384;
    const uint32_t MaxRoadID = 10240;
    const uint32_t MaxJunctionID = MaxObjectID - MaxRoadID;

    const uint32_t MaxRoadVertices = 1 << 24;
    const uint32_t MaxTemporaryVertices = 1 << 18;
    const uint32_t MaxInstancesPerType = 1 << 10;
}