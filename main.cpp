#include <QApplication>
#include <spdlog/spdlog.h>

#include "mainwindow.h"
#include "curve_fitting.h"

int main(int argc, char** argv)
{
    odr::Vec2D startPos{ 0, 0 };
    odr::Vec2D startHdg{ 0, 1 };
    odr::Vec2D endPos{ 1, -1 };
    odr::Vec2D endHdg{ -1, -0.1 };
    endHdg = odr::normalize(endHdg);
    RoadRunner::FitSpiral(startPos, startHdg, endPos, endHdg);
    /*
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

    return app.exec();*/
}