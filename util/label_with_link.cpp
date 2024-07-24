#include "label_with_link.h"

LabelWithLink::LabelWithLink(QUrl url, QString disp, QWidget* parent)
    : QLabel(disp, parent), _url(url) 
{
    setToolTip(url.toString());
    setStyleSheet("font-weight: bold; color: darkblue");
    setAlignment(Qt::AlignCenter);
}

void LabelWithLink::mouseReleaseEvent(QMouseEvent*) 
{
    QDesktopServices::openUrl(_url);
}
