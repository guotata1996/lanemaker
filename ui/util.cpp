#include "util.h"

#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <filesystem>
#include <optional>

namespace RoadRunner
{
    std::string DefaultSaveFolder()
    {
        std::filesystem::path fullPath;
    
#ifdef _WIN32
        fullPath = std::getenv("HOMEDRIVE");
        fullPath /= std::getenv("HOMEPATH");
        fullPath /= "Desktop/saved_map";
#elif __linux__
        fullPath = std::getenv("HOME");
        fullPath /= "Desktop/saved_map";
#else
#endif
        std::filesystem::create_directories(fullPath);
        return fullPath.string();
    }

    std::string CurrentDateTime()
    {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%m-%d_%H-%M-%S");
        return oss.str();
    }

    namespace
    {
        std::optional<std::string> runTimestamp;
    }

    std::string RunTimestamp()
    {
        if (!runTimestamp.has_value())
        {
            runTimestamp.emplace(CurrentDateTime());
        }
        return runTimestamp.value();
    }
}
