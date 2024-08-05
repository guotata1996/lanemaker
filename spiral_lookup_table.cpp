#include "curve_fitting.h"

#include "spdlog/spdlog.h"


int main(int argc, char** argv)
{
    // RoadRunner::WriteFitTable();
    
    auto startPos = odr::Vec2D{ 0, 0 };
    auto startHdg = odr::Vec2D{ 1, 0 };
    auto endPos = odr::Vec2D{ 321, -378 };
    auto endHdg = odr::Vec2D{ 51, -191 };

    auto fit = RoadRunner::FitSpiral(startPos, startHdg, endPos, endHdg);
    if (fit == nullptr)
    {
        return 0;
    }
    if (odr::euclDistance(fit->get_xy(0), startPos) > RoadRunner::SpiralPosPrecision)
    {
        spdlog::error("startPos not match");
    }
    if (odr::euclDistance(fit->get_xy(fit->length), endPos) > RoadRunner::SpiralPosPrecision)
    {
        spdlog::error("endPos not match");
    }
    if (std::abs(odr::angle(fit->get_grad(0), startHdg)) > RoadRunner::SpiralHdgPrecision)
    {
        spdlog::error("startHdg not match");
    }
    if (std::abs(odr::angle(fit->get_grad(fit->length), endHdg)) > RoadRunner::SpiralHdgPrecision)
    {
        spdlog::error("endHdg not match");
    }
    spdlog::info("OK: length = {}", fit->length);
}