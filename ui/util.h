#pragma once
#include <string>

namespace RoadRunner
{
    std::string DefaultSaveFolder();
    
    std::string CurrentDateTime();

    /*Unique per run*/
    std::string RunTimestamp();
}
