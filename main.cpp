#include <QApplication>
#include <spdlog/spdlog.h>

#include "mainwindow.h"


int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    app.setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);

    #ifdef __linux__
        QFont font = QApplication::font() ;
        font.setPointSize(8);
        app.setFont(font);
    #endif

    MainWindow window;
    window.show();

    return app.exec();
}