#include "CreateRoadOptionWidget.h"
#include <QVBoxLayout>
#include <qevent.h>
#include <qpainter.h>

#include <spdlog/spdlog.h>

RoadRunner::SectionProfile activeLeftSetting { 1, 1 };
RoadRunner::SectionProfile activeRightSetting{ -1, 1 };

QSize CreateRoadOptionWidget::sizeHint() const
{
    return QSize(500, 20);
}

void CreateRoadOptionWidget::paintEvent(QPaintEvent* evt)
{
    QWidget::paintEvent(evt);

    QRect rect = evt->rect();

    QPainter painter(this);
    const int RectGap = 2;

    YCenter = rect.y() + rect.height() / 2;
    XLeft = rect.x() + rect.width() / 20;
    XRight = rect.x() + rect.width() / 20 * 19;
    XCenter = rect.x() + rect.width() / 2;
    // Draw background
    painter.setBrush(QColor(210, 210, 210));
    painter.setPen(Qt::NoPen);
    painter.drawRect(0, 0, rect.width() / 2, rect.height());
    // Draw line
    QPen colorPen;
    painter.setPen(colorPen);
    painter.drawLine(XLeft, YCenter, XRight, YCenter);
    // Draw ticks
    TickInterval = static_cast<float>(XRight - XLeft) / (2 * SingleSideLaneLimit);
    const int TickHeight = rect.height() / 2;
    for (int tick = -SingleSideLaneLimit; tick <= SingleSideLaneLimit; ++tick)
    {
        int TickX = XCenter + TickInterval * tick;
        painter.drawLine(TickX, YCenter - TickHeight / 2, TickX, YCenter + TickHeight / 2);
    }

    int lOuterResult = XCenter - static_cast<float>(activeLeftSetting.offsetx2 + activeLeftSetting.laneCount * 2) * TickInterval;
    int lInnerResult = XCenter - static_cast<float>(activeLeftSetting.offsetx2) * TickInterval;
    int rOuterResult = XCenter + static_cast<float>(-activeRightSetting.offsetx2 + activeRightSetting.laneCount * 2) * TickInterval; 
    int rInnerResult = XCenter + static_cast<float>(-activeRightSetting.offsetx2) * TickInterval;

    if (dragIndex == -1)
    {
        handleX[0] = lOuterResult;
        handleX[1] = lInnerResult;
        handleX[2] = rInnerResult;
        handleX[3] = rOuterResult;
    }

    // Draw result
    colorPen.setWidth(5);
    colorPen.setColor(Qt::red);
    painter.setPen(colorPen);
    painter.drawLine(lOuterResult, YCenter, lInnerResult, YCenter);
    
    colorPen.setColor(Qt::green);
    painter.setPen(colorPen);
    painter.drawLine(rOuterResult, YCenter, rInnerResult, YCenter);

    colorPen.setColor(Qt::black);
    painter.setPen(colorPen);

    painter.drawText(TickInterval / 2, YCenter, QString::fromStdString(std::to_string(activeLeftSetting.laneCount)));
    painter.drawText(XRight + TickInterval / 2, YCenter, QString::fromStdString(std::to_string(activeRightSetting.laneCount)));

    // Draw Handles
    colorPen.setColor(Qt::darkYellow);
    painter.setPen(colorPen);

    decltype(handleX) handleXCopy(handleX);
    std::sort(handleXCopy.begin(), handleXCopy.end());
    for (int handle : handleXCopy)
    {
        painter.drawLine(handle, YCenter - TickHeight / 2, handle, YCenter + TickHeight / 2);
    }
}

void CreateRoadOptionWidget::mousePressEvent(QMouseEvent* evt)
{
    if (std::abs(evt->y() - YCenter) < TickInterval / 2 && evt->button() == Qt::MouseButton::LeftButton)
    {
        for (int i = 0; i != handleX.size(); ++i)
        {
            if (std::abs(evt->x() - handleX[i]) < TickInterval / 2)
            {
                dragIndex = i;
                break;
            }
        }
    }
}

void CreateRoadOptionWidget::mouseMoveEvent(QMouseEvent* evt)
{
    if (dragIndex < 0) return;
    auto dragX = static_cast<int>(evt->localPos().x());
    dragX = std::max(XLeft, dragX);
    dragX = std::min(XRight - 1, dragX);
    handleX[dragIndex] = dragX;

    decltype(handleX) handleXCopy(handleX);
    std::sort(handleXCopy.begin(), handleXCopy.end());

    activeLeftSetting.offsetx2 = std::round(static_cast<float>(XCenter - handleXCopy[1]) / TickInterval);
    activeLeftSetting.laneCount = std::round(static_cast<float>(handleXCopy[1] - handleXCopy[0]) / TickInterval / 2);

    activeRightSetting.offsetx2 = std::round(static_cast<float>(XCenter - handleXCopy[2]) / TickInterval);
    activeRightSetting.laneCount = std::round(static_cast<float>(handleXCopy[3] - handleXCopy[2]) / TickInterval / 2);

    update();
}

void CreateRoadOptionWidget::mouseReleaseEvent(QMouseEvent* evt)
{
    if (evt->button() == Qt::MouseButton::LeftButton)
    {
        // Quit drag
        dragIndex = -1;
        update();
    }
}