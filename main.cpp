#include <QApplication>

#include "mainwindow.h"

#include "Geometries/CubicSpline.h"
#include "spdlog/spdlog.h"

int main(int argc, char** argv)
{
    odr::CubicSpline sp;
    const double spLen = 10;
    sp.s0_to_poly.emplace(0, odr::Poly3(0, 2, 0.1, 0.06, 0.001));
    sp.s0_to_poly.emplace(4, odr::Poly3(4, -3, 0.4, -0.01, -0.002));

    odr::CubicSpline sp2;
    sp2.s0_to_poly.emplace(0, odr::Poly3(0, 1, 0.13, -0.06, 0.002));
    
    //double y1 = sp.get(x);
    //sp.reverse(spLen);
    //double y2 = sp.get(spLen - x);

    //double y1 = sp.get(7);
    //auto second = sp.split(4);
    //double y2 = second.get(3);

    sp.join(spLen, sp2);
    double y1 = sp2.get(3);
    double y2 = sp.get(13);
    spdlog::info("Y = {}; Y2 = {}", y1, y2);

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