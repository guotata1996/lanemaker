#include "AnimatedPopupDialog.h"
#include <QApplication>
#include <QDesktopWidget>
#include <QScreen>
#include <QVBoxLayout>

AnimatedPopupDialog::AnimatedPopupDialog(QSize _endSize, QWidget *parent)
    : endSize(_endSize), QDialog(parent) {
    resize(endSize),
    setWindowOpacity(0.1);
}

void AnimatedPopupDialog::enableDimming(bool enabled) {
    useDimming = enabled;
}

void AnimatedPopupDialog::createDimmingOverlay() {
    if (!parentWidget()) return;

    dimmingOverlay = new QWidget(parentWidget());
    dimmingOverlay->setStyleSheet("background-color: rgba(0, 0, 0, 128);");
    dimmingOverlay->setGeometry(parentWidget()->rect());
    dimmingOverlay->show();
    dimmingOverlay->lower(); // Keep below dialog

    auto *overlayEffect = new QGraphicsOpacityEffect(dimmingOverlay);
    dimmingOverlay->setGraphicsEffect(overlayEffect);

    QPropertyAnimation *fade = new QPropertyAnimation(overlayEffect, "opacity");
    fade->setDuration(DurationMS);
    fade->setStartValue(0);
    fade->setEndValue(1);
    fade->start(QAbstractAnimation::DeleteWhenStopped);
}

void AnimatedPopupDialog::showEvent(QShowEvent* e)
{
    requestedClose = false;
    showAnimated();
}

void AnimatedPopupDialog::showAnimated() {
    if (useDimming) createDimmingOverlay();

    QVariantAnimation *fadeIn = new QVariantAnimation(this);
    fadeIn->setDuration(DurationMS);
    fadeIn->setStartValue(0.1);
    fadeIn->setEndValue(1.0);

    connect(fadeIn, &QVariantAnimation::valueChanged, this, [this](QVariant v) {
        setWindowOpacity(v.toDouble()); });

    QPropertyAnimation *posAnim = new QPropertyAnimation(this, "pos");
    posAnim->setDuration(DurationMS);
    posAnim->setStartValue(QPoint(
        parentWidget()->geometry().center().x() - endSize.width() / 2, 
        parentWidget()->geometry().bottom()));
    posAnim->setEndValue(parentWidget()->geometry().center() - QPoint(endSize.width()/2, endSize.height()/2));
    posAnim->setEasingCurve(QEasingCurve::OutBack);

    QParallelAnimationGroup *group = new QParallelAnimationGroup;
    group->addAnimation(fadeIn);
    group->addAnimation(posAnim);

    show();
    group->start(QAbstractAnimation::DeleteWhenStopped);
}

void AnimatedPopupDialog::closeEvent(QCloseEvent* e)
{
    if (!requestedClose)
    {
        closeAnimated();
        e->ignore();
    }
    else
    {
        e->accept();
    }
}

void AnimatedPopupDialog::keyPressEvent(QKeyEvent* e)
{
    if (e->key() == Qt::Key_Escape)
    {
        e->accept();
        closeAnimated();
    }
    else
    {
        QDialog::keyPressEvent(e);
    }
}

void AnimatedPopupDialog::closeAnimated() 
{
    requestedClose = true;

    QVariantAnimation* fadeOut = new QVariantAnimation(this);
    fadeOut->setDuration(DurationMS);
    fadeOut->setStartValue(1.0);
    fadeOut->setEndValue(0.1);

    connect(fadeOut, &QVariantAnimation::valueChanged, this, [this](QVariant v) {
        setWindowOpacity(v.toDouble()); });

    QPropertyAnimation *posAnim = new QPropertyAnimation(this, "pos");
    posAnim->setDuration(DurationMS);
    posAnim->setStartValue(pos());
    posAnim->setEndValue(QPoint(
        parentWidget()->geometry().center().x() - endSize.width() / 2,
        parentWidget()->geometry().bottom()));
    posAnim->setEasingCurve(QEasingCurve::OutBack);

    QParallelAnimationGroup *group = new QParallelAnimationGroup;
    group->addAnimation(posAnim);

    connect(group, &QParallelAnimationGroup::finished, this, &QDialog::close);
    group->start(QAbstractAnimation::DeleteWhenStopped);

    // Optional: fade out dimming overlay
    if (dimmingOverlay) {
        auto *overlayEffect = static_cast<QGraphicsOpacityEffect *>(dimmingOverlay->graphicsEffect());
        QPropertyAnimation *dimOut = new QPropertyAnimation(overlayEffect, "opacity");
        dimOut->setDuration(DurationMS);
        dimOut->setStartValue(1);
        dimOut->setEndValue(0);
        connect(dimOut, &QPropertyAnimation::finished, dimmingOverlay, &QWidget::deleteLater);
        dimOut->start(QAbstractAnimation::DeleteWhenStopped);
    }
}
