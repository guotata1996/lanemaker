#pragma once

#include <QtWidgets>

class LabelWithLink : public QLabel {
    Q_OBJECT
public:
    LabelWithLink(QUrl url, QString disp, QWidget* parent = nullptr);

protected:
    void mouseReleaseEvent(QMouseEvent*);

private:
    QUrl _url;
};
