#include "curve_fitting.h"

#include <map>
#include <fstream>

#include <cereal/archives/json.hpp>
#include <cereal/types/map.hpp>
#include "Geometries/Arc.h"
#include "Geometries/Spiral.h"
#include "spdlog/spdlog.h"


int main(int argc, char** argv)
{
    //spdlog::set_level(spdlog::level::err);

    RoadRunner::WriteFitTable();
    // auto res = RoadRunner::FitSpiral(29, 94);
    //if (res == nullptr)
    //{
    //    spdlog::error("Cannot fit");
    //}
    //return 0;
}