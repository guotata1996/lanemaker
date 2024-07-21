#pragma once
#include <string>
#include <filesystem>

namespace RoadRunner
{
    std::filesystem::path DefaultSaveFolder();
    
    std::string CurrentDateTime();

    /*Unique per run*/
    std::string RunTimestamp();
}
