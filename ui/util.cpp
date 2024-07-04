#include "util.h"

#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <filesystem>

namespace RoadRunner
{
    std::string DefaultSaveFolder()
    {
        auto homeDrive = std::string(std::getenv("HOMEDRIVE"));
        auto homePath = std::string(std::getenv("HOMEPATH"));
        auto fullPath = homeDrive + homePath + "\\Desktop\\saved_map";
        std::filesystem::create_directories(fullPath);
        return fullPath;
    }

    std::string CurrentDateTime()
    {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%m-%d_%H-%M-%S");
        return oss.str();
    }
}