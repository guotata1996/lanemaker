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
}