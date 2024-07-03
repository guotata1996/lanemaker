#include "util.h"

namespace RoadRunner
{
    std::string DefaultSaveFolder()
    {
        auto homeDrive = std::string(std::getenv("HOMEDRIVE"));
        auto homePath = std::string(std::getenv("HOMEPATH"));
        return homeDrive + homePath + "\\Desktop\\saved_map";
    }
}