#include "CreateRoadOptionWidget.h"
#include "action_manager.h"

#include <QVBoxLayout>
#include <qevent.h>
#include <qpainter.h>

#include <spdlog/spdlog.h>
#include <sstream>

SectionProfileConfigWidget* g_createRoadOption;

CreateRoadOptionWidget::CreateRoadOptionWidget():
    rightLogo(":/icons/car_leaving.png"),
    leftLogo(":/icons/car_coming.png")
{
    SetOption(RoadRunner::SectionProfile{ 1, 1 }, RoadRunner::SectionProfile{ -1, 1 });
}

void CreateRoadOptionWidget::showEvent(QShowEvent* event)
{
    emit OptionChangedByUser(activeLeftSetting, activeRightSetting);
    QWidget::showEvent(event);
}

void CreateRoadOptionWidget::SetOption(const RoadRunner::SectionProfile& l, const RoadRunner::SectionProfile& r)
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
    update();
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

    lOuterResult = XCenter - static_cast<float>(activeLeftSetting.offsetx2 + activeLeftSetting.laneCount * 2) * TickInterval;
    lInnerResult = XCenter - static_cast<float>(activeLeftSetting.offsetx2) * TickInterval;
    rOuterResult = XCenter + static_cast<float>(-activeRightSetting.offsetx2 + activeRightSetting.laneCount * 2) * TickInterval; 
    rInnerResult = XCenter + static_cast<float>(-activeRightSetting.offsetx2) * TickInterval;

    if (changedExternally)
    {
        handleX[0] = lOuterResult;
        handleX[1] = lInnerResult;
        handleX[2] = rInnerResult;
        handleX[3] = rOuterResult;
        changedExternally = false;
    }

    // Draw result
    colorPen.setWidth(5);
    colorPen.setColor(Qt::yellow);
    painter.setPen(colorPen);
    painter.drawLine(lOuterResult, YCenter, lInnerResult, YCenter);
    for (int i = 0; i != activeLeftSetting.laneCount; ++i)
    {
        int logoCenter = lInnerResult - (i * 2 + 1) * TickInterval;
        QRectF rect(logoCenter - TickHeight / 2, YCenter - TickHeight, TickHeight, TickHeight);
        painter.drawImage(rect, leftLogo);
    }
    
    colorPen.setColor(Qt::red);
    painter.setPen(colorPen);
    painter.drawLine(rOuterResult, YCenter, rInnerResult, YCenter);
    for (int i = 0; i != activeRightSetting.laneCount; ++i)
    {
        int logoCenter = rInnerResult + (i * 2 + 1) * TickInterval;
        QRectF rect(logoCenter - TickHeight / 2, YCenter - TickHeight, TickHeight, TickHeight);
        painter.drawImage(rect, rightLogo);
    }

    colorPen.setColor(Qt::black);
    painter.setPen(colorPen);

    painter.drawText(TickInterval / 2, YCenter, QString::fromStdString(std::to_string(activeLeftSetting.laneCount)));
    painter.drawText(XRight + TickInterval / 2, YCenter, QString::fromStdString(std::to_string(activeRightSetting.laneCount)));

    // Draw Handles
    for (int i = 0; i != handleX.size(); ++i)
    {
        auto handle = handleX[i];
        colorPen.setColor(dragIndex.empty() ||
            std::find(dragIndex.begin(), dragIndex.end(), i) == dragIndex.end() ?
            Qt::blue : Qt::green);
        painter.setPen(colorPen);
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
                dragIndex.push_back(i);
                handleOrigin.push_back(handleX[i]);
                dragOrigin = evt->x();
                update();
                return;
            }
        }

        if (lOuterResult < evt->x() && evt->x() < lInnerResult)
        {
            std::sort(handleX.begin(), handleX.end());
            dragIndex.push_back(0);
            dragIndex.push_back(1);
            handleOrigin.push_back(handleX[0]);
            handleOrigin.push_back(handleX[1]);
            dragOrigin = evt->x();
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
            update();
            return;
        }
    }
}

void CreateRoadOptionWidget::mouseMoveEvent(QMouseEvent* evt)
{
    if (dragIndex.empty()) return;

    int dragDelta = evt->localPos().x() - dragOrigin;
    for (int i = 0; i != dragIndex.size(); ++i)
    {
        int dragX = handleOrigin[i] + dragDelta;
        dragX = std::max(XLeft, dragX);
        dragX = std::min(XRight - 1, dragX);
        int tickAt = std::round((dragX - XCenter) / TickInterval);
        dragX = tickAt * TickInterval + XCenter;
        handleX[dragIndex[i]] = dragX;
    }


    decltype(handleX) handleXCopy(handleX);
    std::sort(handleXCopy.begin(), handleXCopy.end());

    activeLeftSetting.offsetx2 = std::round(static_cast<float>(XCenter - handleXCopy[1]) / TickInterval);
    activeLeftSetting.laneCount = std::floor(static_cast<int>(std::round(static_cast<float>(handleXCopy[1] - handleXCopy[0]) / TickInterval)) / 2);

    activeRightSetting.offsetx2 = std::round(static_cast<float>(XCenter - handleXCopy[2]) / TickInterval);
    activeRightSetting.laneCount = std::floor(static_cast<int>(std::round(static_cast<float>(handleXCopy[3] - handleXCopy[2]) / TickInterval)) / 2);

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

CreateLaneOptionWidget::CreateLaneOptionWidget()
{
    auto mainLayout = new QHBoxLayout;
    leftSlider = new QSlider(Qt::Horizontal);
    leftSlider->setRange(0, 5);
    leftSlider->setTickPosition(QSlider::TicksBelow);
    rightSlider = new QSlider(Qt::Horizontal);
    rightSlider->setRange(1, 5);
    rightSlider->setTickPosition(QSlider::TicksBelow);
    resultLabel = new QLabel("0|1");
    resultLabel->setFont(QFont("Helvetica", 14));
    auto leftLogo = new QLabel;
    leftLogo->setPixmap(QPixmap(":/icons/car_coming.png").scaledToWidth(20));
    auto rightLogo = new QLabel;
    rightLogo->setPixmap(QPixmap(":/icons/car_leaving.png").scaledToWidth(20));
    mainLayout->addWidget(leftSlider);
    mainLayout->addWidget(leftLogo);
    mainLayout->addWidget(resultLabel);
    mainLayout->addWidget(rightLogo);
    mainLayout->addWidget(rightSlider);
    setLayout(mainLayout);

    connect(leftSlider, &QSlider::valueChanged, this, &CreateLaneOptionWidget::updateLabel);
    connect(rightSlider, &QSlider::valueChanged, this, &CreateLaneOptionWidget::updateLabel);
}

RoadRunner::SectionProfile CreateLaneOptionWidget::LeftResult() const
{
    return RoadRunner::SectionProfile{ 0, static_cast<RoadRunner::type_t>(leftSlider->value()) };
}

RoadRunner::SectionProfile CreateLaneOptionWidget::RightResult() const
{
    return RoadRunner::SectionProfile{ 0, static_cast<RoadRunner::type_t>(rightSlider->value()) };
}

void CreateLaneOptionWidget::showEvent(QShowEvent* event)
{
    emit OptionChangedByUser(LeftResult(), RightResult());
    QWidget::showEvent(event);
}

void CreateLaneOptionWidget::SetOption(const RoadRunner::SectionProfile& l, const RoadRunner::SectionProfile& r)
{
    const QSignalBlocker blocker(this);
    leftSlider->setValue(l.laneCount);
    rightSlider->setValue(r.laneCount);
}

void CreateLaneOptionWidget::updateLabel()
{
    std::stringstream ss;
    ss << "" << leftSlider->value() << "|" << rightSlider->value();
    resultLabel->setText(QString::fromStdString(ss.str()));

    emit OptionChangedByUser(LeftResult(), RightResult());
}

SectionProfileConfigWidget::SectionProfileConfigWidget():
    roadMode(new CreateRoadOptionWidget),
    laneMode(new CreateLaneOptionWidget)
{
    addWidget(roadMode);
    addWidget(laneMode);
    setMaximumHeight(40);

    connect(roadMode, &CreateRoadOptionWidget::OptionChangedByUser, 
        this, &SectionProfileConfigWidget::OptionChangedOnPage);
    connect(laneMode, &CreateLaneOptionWidget::OptionChangedByUser,
        this, &SectionProfileConfigWidget::OptionChangedOnPage);
}

void SectionProfileConfigWidget::OptionChangedOnPage(RoadRunner::SectionProfile left, RoadRunner::SectionProfile right)
{
    if (leftProfileSetting != left || rightProfileSetting != right)
    {
        RoadRunner::ActionManager::Instance()->Record(left, right);
    }
    leftProfileSetting = left;
    rightProfileSetting = right;
}

void SectionProfileConfigWidget::SetOption(const RoadRunner::SectionProfile& l, const RoadRunner::SectionProfile& r)
{
    roadMode->SetOption(l, r);
    laneMode->SetOption(l, r);
    OptionChangedOnPage(l, r);
}

QSize SectionProfileConfigWidget::sizeHint() const
{
    return QSize(500, 20);
}

void SectionProfileConfigWidget::GotoRoadMode()
{
    show();
    setCurrentIndex(0);
}

void SectionProfileConfigWidget::GotoLaneMode()
{
    show();
    setCurrentIndex(1);
}