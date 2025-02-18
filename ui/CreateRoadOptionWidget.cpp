#include "CreateRoadOptionWidget.h"
#include "action_manager.h"

#include <QHBoxLayout>
#include <qevent.h>
#include <qpainter.h>

SectionProfileConfigWidget* g_createRoadOption;

CreateRoadOptionWidget::CreateRoadOptionWidget():
    rightLogo(":/icons/car_leaving.png"),
    leftLogo(":/icons/car_coming.png")
{
    setMinimumWidth(400);
    setMinimumHeight(50);
    Reset();
}

void CreateRoadOptionWidget::Reset()
{
    SetOption(LM::LanePlan{ 1, 1 }, LM::LanePlan{ -1, 1 });
}

void CreateRoadOptionWidget::SetMode(bool roadMode)
{
    if (!roadMode)
    {
        activeRightSetting.laneCount = std::max((int8_t)1, activeRightSetting.laneCount);
    }
    changedExternally = true;
    update();
}

void CreateRoadOptionWidget::showEvent(QShowEvent* event)
{
    emit OptionChangedByUser(activeLeftSetting, activeRightSetting);
    QWidget::showEvent(event);
}

void CreateRoadOptionWidget::SetOption(const LM::LanePlan& l, const LM::LanePlan& r)
{
    const QSignalBlocker blocker(this);
    activeLeftSetting = l;
    activeRightSetting = r;
    if (activeLeftSetting.laneCount == 0)
    {
        activeLeftSetting.offsetx2 = activeRightSetting.offsetx2;
    }
    if (activeRightSetting.laneCount == 0)
    {
        activeRightSetting.offsetx2 = activeLeftSetting.offsetx2;
    }
    changedExternally = true;
    repaint();
}

void CreateRoadOptionWidget::resizeEvent(QResizeEvent* event)
{
    changedExternally = true;
    QWidget::resizeEvent(event);
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
    // Draw ruler
    TickInterval = static_cast<float>(XRight - XLeft) / (2 * SingleSideLaneLimit);
    QPen colorPen;
    painter.setPen(colorPen);
    painter.drawLine(XCenter - SingleSideLaneLimit * TickInterval, YCenter, 
                     XCenter + SingleSideLaneLimit * TickInterval, YCenter);
    // Draw ticks
    const int TickHeight = rect.height() / 2;
    for (int tick = -SingleSideLaneLimit; tick <= SingleSideLaneLimit; ++tick)
    {
        int TickX = XCenter + TickInterval * tick;
        painter.drawLine(TickX, YCenter - TickHeight / 2, TickX, YCenter + TickHeight / 2);
    }

    auto lOffsetx2 = activeLeftSetting.offsetx2;
    auto rOffsetx2 = activeRightSetting.offsetx2;

    lOuterResult = XCenter - static_cast<float>(lOffsetx2 + activeLeftSetting.laneCount * 2) * TickInterval;
    lInnerResult = XCenter - static_cast<float>(lOffsetx2) * TickInterval;
    rOuterResult = XCenter + static_cast<float>(-rOffsetx2 + activeRightSetting.laneCount * 2) * TickInterval;
    rInnerResult = XCenter + static_cast<float>(-rOffsetx2) * TickInterval;

    if (changedExternally)
    {
        handleX[0] = lOuterResult;
        handleX[1] = lInnerResult;
        handleX[2] = rInnerResult;
        handleX[3] = rOuterResult;
        changedExternally = false;
    }

    // Draw result
    colorPen.setWidth(3);
    if (activeLeftSetting.laneCount != 0)
    {
        colorPen.setColor(Qt::red);
        painter.setPen(colorPen);
        painter.drawLine(lOuterResult, YCenter, lInnerResult, YCenter);
        for (int i = 0; i != activeLeftSetting.laneCount; ++i)
        {
            int logoCenter = lInnerResult - (i * 2 + 1) * TickInterval;
            QRectF rect(logoCenter - TickHeight / 2, YCenter - TickHeight, TickHeight, TickHeight);
            painter.drawImage(rect, leftLogo);
        }
    }
    
    if (activeRightSetting.laneCount != 0)
    {
        colorPen.setColor(Qt::green);
        painter.setPen(colorPen);
        painter.drawLine(rOuterResult, YCenter, rInnerResult, YCenter);
        for (int i = 0; i != activeRightSetting.laneCount; ++i)
        {
            int logoCenter = rInnerResult + (i * 2 + 1) * TickInterval;
            QRectF rect(logoCenter - TickHeight / 2, YCenter - TickHeight, TickHeight, TickHeight);
            painter.drawImage(rect, rightLogo);
        }
    }

    // Draw Handles
    colorPen.setWidth(5);
    colorPen.setColor(Qt::black);
    painter.setPen(colorPen);

    for (int i = 0; i != handleX.size(); ++i)
    {
        if ((i == 0 || i == 1) && handleX[0] == handleX[1]) continue;
        if ((i == 2 || i == 3) && handleX[2] == handleX[3]) continue;

        auto handle = handleX[i];
        colorPen.setColor(dragIndex.empty() ||
            std::find(dragIndex.begin(), dragIndex.end(), i) == dragIndex.end() ?
            Qt::gray : Qt::blue);
        painter.setPen(colorPen);
        painter.drawLine(handle, YCenter - TickHeight / 2, handle, YCenter + TickHeight / 2);
    }
}

void CreateRoadOptionWidget::mousePressEvent(QMouseEvent* evt)
{
    if (evt->button() == Qt::MouseButton::LeftButton)
    {
        if (lOuterResult < evt->x() && evt->x() < lInnerResult)
        {
            std::sort(handleX.begin(), handleX.end());
            dragIndex.push_back(0);
            dragIndex.push_back(1);
            handleOrigin.push_back(handleX[0]);
            handleOrigin.push_back(handleX[1]);
            dragOrigin = evt->x();
            dragLimit = std::abs(handleX[1] - handleX[2]);
            update();
            return;
        }

        if (rInnerResult < evt->x() && evt->x() < rOuterResult)
        {
            dragIndex.push_back(2);
            dragIndex.push_back(3);
            handleOrigin.push_back(handleX[2]);
            handleOrigin.push_back(handleX[3]);
            dragOrigin = evt->x();
            dragLimit = std::abs(handleX[1] - handleX[2]);

            update();
            return;
        }
    }
}

void CreateRoadOptionWidget::mouseMoveEvent(QMouseEvent* evt)
{
    if (dragIndex.empty()) return;

    bool draggingLeft = dragIndex[0] == 0;
    int dragDelta = evt->localPos().x() - dragOrigin;
    if (draggingLeft && handleX[2] != handleX[3])
    {
        dragDelta = std::min(dragDelta, dragLimit);
    }
    else if (!draggingLeft & handleX[0] != handleX[1])
    {
        dragDelta = std::max(dragDelta, -dragLimit);
    }
    if (dragDelta == 0) return;

    for (int i = 0; i != dragIndex.size(); ++i)
    {
        int dragX = handleOrigin[i] + dragDelta;
        dragX = std::max(XLeft, dragX);
        dragX = std::min(XRight - 1, dragX);
        int tickAt = std::round((dragX - XCenter) / TickInterval);
        dragX = tickAt * TickInterval + XCenter;
        handleX[dragIndex[i]] = dragX;
    }

    activeLeftSetting.offsetx2 = std::round(static_cast<float>(XCenter - handleX[1]) / TickInterval);
    activeLeftSetting.laneCount = std::floor(static_cast<int>(std::round(static_cast<float>(handleX[1] - handleX[0]) / TickInterval)) / 2);

    activeRightSetting.offsetx2 = std::round(static_cast<float>(XCenter - handleX[2]) / TickInterval);
    activeRightSetting.laneCount = std::floor(static_cast<int>(std::round(static_cast<float>(handleX[3] - handleX[2]) / TickInterval)) / 2);

    update();
}

void CreateRoadOptionWidget::mouseReleaseEvent(QMouseEvent* evt)
{
    if (evt->button() == Qt::MouseButton::LeftButton)
    {
        // Quit drag
        dragIndex.clear();
        handleOrigin.clear();
        update();

        emit OptionChangedByUser(activeLeftSetting, activeRightSetting);
    }
}

SectionProfileConfigWidget::SectionProfileConfigWidget():
    visual(new CreateRoadOptionWidget),
    leftMinus(new QToolButton), leftPlus(new QToolButton),
    rightMinus(new QToolButton), rightPlus(new QToolButton),
    incLogo(":/icons/add.png"), decLogo(":/icons/minus.png")
{
    setMinimumWidth(550); 
    setMaximumWidth(850);
    QHBoxLayout* layout = new QHBoxLayout(this);
    
    int size = style()->pixelMetric(QStyle::PM_ToolBarIconSize);
    QSize iconSize(size, size);
    leftMinus->setIcon(decLogo);
    leftMinus->setIconSize(iconSize);
    leftPlus->setIcon(incLogo);
    leftPlus->setIconSize(iconSize);
    rightMinus->setIcon(decLogo);
    rightMinus->setIconSize(iconSize);
    rightPlus->setIcon(incLogo);
    rightPlus->setIconSize(iconSize);
    layout->addWidget(leftMinus);
    layout->addWidget(leftPlus);
    layout->addWidget(visual);
    visual->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    layout->addWidget(rightMinus);
    layout->addWidget(rightPlus);

    connect(visual, &CreateRoadOptionWidget::OptionChangedByUser, this, &SectionProfileConfigWidget::OnOptionChange);
    connect(leftMinus, &QAbstractButton::clicked, [this]()
        {
            auto lPlan = LeftResult();
            const auto rPlan = RightResult();
            if (lPlan.laneCount > 1 || lPlan.laneCount == 1 && rPlan.laneCount != 0)
            {
                lPlan.laneCount--;
                SetOption(lPlan, RightResult());
            }
        });
    
    connect(leftPlus, &QAbstractButton::clicked, [this]()
        {
            auto lPlan = LeftResult();
            const auto rPlan = RightResult();
            if (lPlan.laneCount < 4)
            {
                if (lPlan.laneCount == 0)
                    lPlan.offsetx2 = std::max((int8_t)0, rPlan.offsetx2);
                lPlan.laneCount++;
                SetOption(lPlan, RightResult());
            }
        });
    connect(rightMinus, &QAbstractButton::clicked, [this]()
        {
            auto rPlan = RightResult();
            const auto lPlan = LeftResult();
            if (rPlan.laneCount > 1 || rPlan.laneCount == 1 && lPlan.laneCount != 0)
            {
                rPlan.laneCount--;
                SetOption(LeftResult(), rPlan);
            }
        });
    connect(rightPlus, &QAbstractButton::clicked, [this]()
        {
            auto rPlan = RightResult();
            const auto lPlan = LeftResult();
            if (rPlan.laneCount < 4)
            {
                if (rPlan.laneCount == 0)
                    rPlan.offsetx2 = std::min((int8_t)0, lPlan.offsetx2);
                rPlan.laneCount++;
                SetOption(LeftResult(), rPlan);
            }
        });
    
    setLayout(layout);
}

void SectionProfileConfigWidget::Reset()
{
    visual->Reset();
}

void SectionProfileConfigWidget::OnOptionChange(LM::LanePlan left, LM::LanePlan right)
{
    LM::ActionManager::Instance()->Record(left, right);
}

void SectionProfileConfigWidget::SetOption(const LM::LanePlan& l, const LM::LanePlan& r)
{
    if (LeftResult() == l && RightResult() == r)
    {
        return;
    }
    visual->SetOption(l, r);
    OnOptionChange(l, r);
}

QSize SectionProfileConfigWidget::sizeHint() const
{
    return QSize(500, 60);
}

void SectionProfileConfigWidget::GotoRoadMode()
{
    show();
    visual->SetMode(true);
}

void SectionProfileConfigWidget::GotoLaneMode()
{
    show();
    visual->SetMode(false);
}