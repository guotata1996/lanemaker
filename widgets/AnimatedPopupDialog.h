#ifndef ANIMATEDPOPUPDIALOG_H
#define ANIMATEDPOPUPDIALOG_H

#include <QDialog>
#include <QGraphicsOpacityEffect>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>
#include <QShowEvent>
#include <QCloseEvent>
#include <QKeyEvent>

class AnimatedPopupDialog : public QDialog {
    Q_OBJECT
public:
    explicit AnimatedPopupDialog(QSize endSize, 
        bool useDimming = true, 
        QWidget *parent = nullptr);

protected:
    void showEvent(QShowEvent*) override;

    void closeEvent(QCloseEvent*) override;
    void keyPressEvent(QKeyEvent*) override;

private:
    void showAnimated();
    void closeAnimated();

    const int DurationMS = 500;
    QSize endSize;
    QWidget* dimmingOverlay = nullptr;
    bool useDimming = true;

    void createDimmingOverlay();

    bool requestedClose = false;
};

#endif // ANIMATEDPOPUPDIALOG_H
