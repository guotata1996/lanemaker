#include "util.h"

#include <iomanip>
#include <ctime>
#include <sstream>
#include <optional>

namespace RoadRunner
{
    std::filesystem::path DefaultSaveFolder()
    {
        std::filesystem::path fullPath;
    
#ifdef _WIN32
        fullPath = std::getenv("HOMEDRIVE");
        fullPath /= std::getenv("HOMEPATH");
        fullPath /= "RoadRunnerData";
#elif __linux__
        fullPath = std::getenv("HOME");
        fullPath /= "RoadRunnerData";
#else
#endif
        bool success = true;
        try
        {
            std::filesystem::create_directories(fullPath);
        }
        catch (std::filesystem::filesystem_error)
        {
            success = false;
        }

        if (!success)
        {
            // Fallback to executable folder
            fullPath = "";
            std::filesystem::create_directories(fullPath);
        }
        
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

    TQDM<std::vector<size_t>>::HelperRange range(size_t s)
    {
        std::vector<size_t> rtn(s);
        for (int i = 0; i != s; ++i)
        {
            rtn[i] = i;
        }
        return rtn;
    }
}
