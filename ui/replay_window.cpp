#include "replay_window.h"
#include <QPushButton>
#include <QFileDialog>
#include <QMenuBar>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QListWidget>
#include <qtimer.h>

#include "action_manager.h"

#include "util.h"

#include "spdlog/spdlog.h"

ReplayWindow::ReplayWindow(QWidget* parent): QDialog(parent)
{
	setWindowTitle("Action Relay");

	QVBoxLayout* layout = new QVBoxLayout(this);
	setLayout(layout);
	listWidget = new QListWidget;
	layout->addWidget(listWidget);

	QHBoxLayout* buttonLayout = new QHBoxLayout();
	auto playPauseButton = new QPushButton(">");
	playPauseButton->setCheckable(true);
	buttonLayout->addWidget(playPauseButton);
	auto singleStepButton = new QPushButton("1");
	buttonLayout->addWidget(singleStepButton);
	resetButton = new QPushButton("|<");
	buttonLayout->addWidget(resetButton);
	layout->addLayout(buttonLayout);

	replayTimer = new QTimer;
	replayTimer->setInterval(1000);

	connect(listWidget, &QListWidget::itemClicked, this, &ReplayWindow::PlaceBreakpointOn);
	connect(replayTimer, &QTimer::timeout, this, &ReplayWindow::SingleStep);
	connect(singleStepButton, &QPushButton::clicked, this, &ReplayWindow::SingleStep);
	connect(playPauseButton, &QPushButton::toggled, this, &ReplayWindow::ToggleAnimatedPlay);
	connect(resetButton, &QPushButton::clicked, this, &ReplayWindow::ReplayFromStart);
}

void ReplayWindow::LoadHistory(std::string fpath)
{
	fullHistory = RoadRunner::ActionManager::Load(fpath);
	FillHistoryTable();
}

void ReplayWindow::FillHistoryTable()
{
	listWidget->clear();
	supported = true;
	for (const auto& action : fullHistory)
	{
		std::string desc;
		QIcon icon;

		switch (action.type)
		{
		case RoadRunner::ActionType::Action_Mouse:
			desc = "Mouse";
			switch (action.detail.mouse.button)
			{
			case Qt::MouseButton::LeftButton:
				desc += " left";
				break;
			case Qt::MouseButton::RightButton:
				desc += " right";
				break;
			case Qt::MouseButton::MidButton:
				desc += " mid";
				break;
			default:
				supported = false;
				break;
			}
			switch (action.detail.mouse.type)
			{
			case QEvent::MouseButtonPress:
				desc += " press";
				break;
			case QEvent::MouseButtonDblClick:
				desc += " double click";
				break;
			case QEvent::MouseMove:
				desc += " move";
				break;
			case QEvent::MouseButtonRelease:
				desc += " release";
				break;
			default:
				supported = false;
				break;
			}
			desc += " (" + std::to_string(action.detail.mouse.x) +
				"," + std::to_string(action.detail.mouse.y) + ")";
			break;
		case RoadRunner::ActionType::Action_KeyPress:
			desc = "KeyPress";
			desc += " " + QKeySequence(action.detail.keyPress.key).toString().toStdString();
			break;
		case RoadRunner::ActionType::Action_Viewport:
			desc = "ViewChange";
			desc += " zoom:" + std::to_string(action.detail.viewport.zoom);
			desc += " rot:" + std::to_string(action.detail.viewport.rotate);
			icon = QIcon(QPixmap(":/icons/zoomin.png"));
			break;
		case RoadRunner::ActionType::Action_Undo:
			desc = "Undo";
			break;
		case RoadRunner::ActionType::Action_Redo:
			desc = "Redo";
			break;
		case RoadRunner::ActionType::Action_ChangeMode:
			desc = "ChangeMode";
			switch (action.detail.changeMode.mode)
			{
			case MapView::Mode_Create:
				desc += " road";
				icon = QIcon(QPixmap(":/icons/road_mode.png"));
				break;
			case MapView::Mode_CreateLanes:
				desc += " lane";
				icon = QIcon(QPixmap(":/icons/lane_mode.png"));
				break;
			case MapView::Mode_Modify:
				desc += " modify";
				icon = QIcon(QPixmap(":/icons/modify_mode.PNG"));
				break;
			case MapView::Mode_Destroy:
				desc += " destroy";
				icon = QIcon(QPixmap(":/icons/destroy_mode.png"));
				break;
			default:
				desc += " none";
				icon = QIcon(QPixmap(":/icons/view_mode.png"));
				break;
			}
			break;
		case RoadRunner::ActionType::Action_ChangeProfile:
			desc = "ChangeProfile";
			desc += " L:lane:" + std::to_string(action.detail.changeProfile.leftProfile.laneCount);
			desc += " offset:" + std::to_string(action.detail.changeProfile.leftProfile.offsetx2);
			desc += " R:lane" + std::to_string(action.detail.changeProfile.rightProfile.offsetx2);
			desc += " offset:" + std::to_string(action.detail.changeProfile.rightProfile.offsetx2);
			break;
		default:
			supported = false;
			break;
		} 

		auto item = new QListWidgetItem(icon, QString(desc.c_str()), listWidget);
		item->setToolTip("Click to toggle breakpoint");
		listWidget->addItem(item);
	}

	nextToReplay = 0;
	if (!fullHistory.empty())
	{
		listWidget->item(nextToReplay)->setBackground(Qt::cyan);
	}
}

void ReplayWindow::PlaceBreakpointOn(QListWidgetItem* item)
{
	int row = listWidget->row(item);
	if (item->data(BreakPointFlag).toBool())
	{
		item->setBackground(QBrush(Qt::NoBrush));
		item->setData(BreakPointFlag, false);
	}
	else
	{
		item->setBackground(QBrush(Qt::yellow));
		item->setData(BreakPointFlag, true);
	}
}

void ReplayWindow::SingleStep()
{
	if (nextToReplay >= fullHistory.size())
	{
		return;
	}

	auto replayingItem = listWidget->item(nextToReplay);
	replayingItem->setBackground(Qt::NoBrush);
	replayingItem->setFlags(replayingItem->flags() & ~Qt::ItemIsEnabled);
	RoadRunner::ActionManager::Instance()->Replay(fullHistory[nextToReplay++]);
	if (nextToReplay < fullHistory.size())
	{
		listWidget->item(nextToReplay)->setBackground(Qt::cyan);
	}
}

void ReplayWindow::ToggleAnimatedPlay(bool play)
{
	if (play)
	{
		replayTimer->start();
		resetButton->setEnabled(false);
	}
	else
	{
		replayTimer->stop();
		resetButton->setEnabled(true);
	}
}

void ReplayWindow::ReplayFromStart()
{
	replayTimer->stop();
	for (int i = 0; i != listWidget->count(); ++i)
	{
		auto item = listWidget->item(i);
		item->setFlags(item->flags() | Qt::ItemIsEnabled);
		item->setBackground(Qt::NoBrush);
	}
	nextToReplay = 0;

	if (!fullHistory.empty())
	{
		listWidget->item(nextToReplay)->setBackground(Qt::cyan);
	}

	emit Restart();
}