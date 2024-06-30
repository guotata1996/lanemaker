#include <QApplication>
#include <spdlog/spdlog.h>

#include "mainwindow.h"


int main(int argc, char** argv)
{
    spdlog::set_level(spdlog::level::info);

    QApplication app(argc, argv);
    app.setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);

    MainWindow window;
    window.show();

    return app.exec();
}