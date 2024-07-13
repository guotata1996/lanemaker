#include "replay_window.h"
#include <QPushButton>
#include <QFileDialog>
#include <QMenuBar>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QListWidget>
#include <qtimer.h>
#include <QCheckBox>

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
	withDelay = new QCheckBox("Tutorial");
	buttonLayout->addWidget(withDelay);
	playPauseButton = new QPushButton(">");
	playPauseButton->setCheckable(true);
	buttonLayout->addWidget(playPauseButton);
	auto singleStepButton = new QPushButton("1");
	buttonLayout->addWidget(singleStepButton);
	resetButton = new QPushButton("|<");
	buttonLayout->addWidget(resetButton);
	layout->addLayout(buttonLayout);

	replayTimer = new QTimer;

	connect(listWidget, &QListWidget::itemClicked, this, &ReplayWindow::PlaceBreakpointOn);
	connect(replayTimer, &QTimer::timeout, this, &ReplayWindow::StepRealDelay);
	connect(singleStepButton, &QPushButton::clicked, this, &ReplayWindow::SingleStep);
	connect(playPauseButton, &QPushButton::toggled, this, &ReplayWindow::ToggleAnimatedPlay);
	connect(resetButton, &QPushButton::clicked, this, &ReplayWindow::ReplayFromStart);
}

void ReplayWindow::LoadHistory(std::string fpath)
{
	fullHistory = RoadRunner::ActionManager::Load(fpath);
	FillHistoryTable();
}

void ReplayWindow::closeEvent(QCloseEvent* e)
{
	playPauseButton->setChecked(false);
	QDialog::closeEvent(e);
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
				switch (action.detail.mouse.button)
				{
				case Qt::MouseButton::LeftButton:
					icon = QIcon(QPixmap(":/icons/LMB.png"));
					break;
				case Qt::MouseButton::RightButton:
					icon = QIcon(QPixmap(":/icons/RMB.png"));
					break;
				case Qt::MouseButton::MidButton:
					icon = QIcon(QPixmap(":/icons/CMB.png"));
					break;
				}
				break;
			case QEvent::MouseButtonDblClick:
				desc += " double click";
				icon = QIcon(QPixmap(":/icons/MouseDblClick.png"));
				break;
			case QEvent::MouseMove:
				desc += " move";
				icon = QIcon(QPixmap(":/icons/MouseMove.png"));
				break;
			case QEvent::MouseButtonRelease:
				desc += " release";
				switch (action.detail.mouse.button)
				{
				case Qt::MouseButton::LeftButton:
					icon = QIcon(QPixmap(":/icons/LMB_Release.png"));
					break;
				case Qt::MouseButton::RightButton:
					icon = QIcon(QPixmap(":/icons/RMB_Release.png"));
					break;
				case Qt::MouseButton::MidButton:
					icon = QIcon(QPixmap(":/icons/CMB_Release.png"));
					break;
				}
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
			icon = QIcon(QPixmap(":/icons/keyboard.png"));
			break;
		case RoadRunner::ActionType::Action_Viewport:
			desc = "ViewChange";
			desc += " zoom:" + std::to_string(action.detail.viewport.zoom);
			desc += " rot:" + std::to_string(action.detail.viewport.rotate);
			icon = QIcon(QPixmap(":/icons/viewport.png"));
			break;
		case RoadRunner::ActionType::Action_Undo:
			icon = QIcon(QPixmap(":/icons/undo.png"));
			desc = "Undo";
			break;
		case RoadRunner::ActionType::Action_Redo:
			icon = QIcon(QPixmap(":/icons/redo.png"));
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
			icon = QIcon(QPixmap(":/icons/car_coming.png"));
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
		listWidget->item(nextToReplay)->setForeground(NextReplayFG);
	}
}

void ReplayWindow::PlaceBreakpointOn(QListWidgetItem* item)
{
	if ((item->flags() & Qt::ItemIsEnabled) == 0)
	{
		return;
	}
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
	replayingItem->setForeground(Qt::NoBrush);
	replayingItem->setFlags(replayingItem->flags() & ~Qt::ItemIsEnabled);
	RoadRunner::ActionManager::Instance()->Replay(fullHistory[nextToReplay++]);
	if (nextToReplay < fullHistory.size())
	{
		listWidget->item(nextToReplay)->setForeground(NextReplayFG);
		listWidget->scrollTo(listWidget->model()->index(nextToReplay, 0));
	}

	if (replayingItem->data(BreakPointFlag).toBool())
	{
		// break point triggered: pause
		if (playPauseButton->isChecked())
		{
			playPauseButton->setChecked(false);
		}
	}
}

void ReplayWindow::StepRealDelay()
{
	if (!withDelay->isChecked() || nextToReplay == 0)
	{
		SingleStep();
		return;
	}
	
	if (nextToReplay >= fullHistory.size())
	{
		return;
	}

	int requiredDelay = fullHistory[nextToReplay].timeMS - fullHistory[nextToReplay - 1].timeMS;
	pausedMS += TimerInterval;
	if (pausedMS >= requiredDelay)
	{
		SingleStep();
		pausedMS = 0;
	}
}

void ReplayWindow::ToggleAnimatedPlay(bool play)
{
	if (play)
	{
		pausedMS = 0;
		replayTimer->setInterval(withDelay->isChecked() ? TimerInterval : 0);
		replayTimer->start();
		resetButton->setEnabled(false);
		withDelay->setEnabled(false);
	}
	else
	{
		replayTimer->stop();
		resetButton->setEnabled(true);
		withDelay->setEnabled(true);
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
		listWidget->item(nextToReplay)->setForeground(NextReplayFG);
	}

	emit Restart();
}