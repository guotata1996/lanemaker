#include <QApplication>

#include "mainwindow.h"

#include "Geometries/CubicSpline.h"
#include "road_profile.h"
#include "spdlog/spdlog.h"

int main(int argc, char** argv)
{
    odr::CubicSpline sp;
    const double spLen = 100;

    odr::CubicSpline sp2;
    sp2.s0_to_poly.emplace(0, odr::Poly3(0, 1, 0.13, -0.06, 0.002));
    
    //double y1 = sp.get(x);
    //sp.reverse(spLen);
    //double y2 = sp.get(spLen - x);

    //double y1 = sp.get(7);
    //auto second = sp.split(4);
    //double y2 = second.get(3);

    //sp.join(spLen, sp2);
    //double y1 = sp2.get(3);
    //double y2 = sp.get(13);
    //spdlog::info("Y = {}; Y2 = {}", y1, y2);

    RoadRunner::CubicSplineGenerator::OverwriteSection(sp, spLen, 40, 40, 5);
    //RoadRunner::CubicSplineGenerator::OverwriteSection(sp, spLen, 0, 10, 10);
    spdlog::info("{}", sp.get(43));
    for (auto k_v : sp.s0_to_poly)
    {
        spdlog::info("Key: {}", k_v.first);
    }

    QApplication app(argc, argv);
    app.setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);
    app.setAttribute(Qt::AA_DisableWindowContextHelpButton);

    #ifdef __linux__
        QFont font = QApplication::font() ;
        font.setPointSize(8);
        app.setFont(font);
    #endif

    MainWindow window;
    window.show();

    return app.exec();
}