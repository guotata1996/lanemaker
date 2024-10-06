#include <QApplication>

#include "mainwindow.h"

int main(int argc, char** argv)
{
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

    if (argc > 1)
    {
        window.runReplay(argv[1]);
        return 0;
    }
    return app.exec();
}