namespace RoadRunner
{
    /*Protect bad (way too long) geometry from freezing UI*/
    const double SingleDrawMaxLength = 500;

    /*Road will be rejected if ref line turns too sharp*/
    const double RoadMaxCurvature = 1.0 / 4;

    /*Consecutive ctrl points can either duplicate (by dbl-clicking), or placed further than this threshold.*/
    const double DupCtrlPointsDist = 0.1;

    const double SnapRadiusPx = 20;        
    
    /*Space for connecting road curvature*/
    const double JunctionTrimMax = 10;
    const double JunctionTrimMin = 2;
}