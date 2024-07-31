#include "replay_window.h"
#include "action_manager.h"
#include "util.h"

#include <QtWidgets>

#include <fstream>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include "spdlog/spdlog.h"

ReplayWindow::ReplayWindow(QWidget* parent): QDialog(parent)
{
	setWindowTitle("Action Relay");

	QVBoxLayout* layout = new QVBoxLayout(this);
	setLayout(layout);
	listWidget = new QListWidget;
	layout->addWidget(listWidget);

	QHBoxLayout* statusLayout = new QHBoxLayout();
	mouseStatus = new QLabel;
	mouseStatus->setPixmap(QPixmap(":/icons/MouseMove.png"));
	statusLayout->addWidget(mouseStatus);
	keyStatus = new QLabel;	
	keyStatus->hide();
	mouseStatus->hide();

	statusLayout->addWidget(keyStatus);
	layout->addLayout(statusLayout);

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

	connect(listWidget, &QListWidget::itemDoubleClicked, this, &ReplayWindow::PlaceBreakpointOn);
	connect(replayTimer, &QTimer::timeout, this, &ReplayWindow::StepRealDelay);
	connect(singleStepButton, &QPushButton::clicked, this, &ReplayWindow::SingleStep);
	connect(playPauseButton, &QPushButton::toggled, this, &ReplayWindow::PlayPause);
	connect(resetButton, &QPushButton::clicked, this, &ReplayWindow::ReplayFromStart);
	connect(withDelay, &QCheckBox::toggled, keyStatus, &QLabel::setVisible);
	connect(withDelay, &QCheckBox::toggled, mouseStatus, &QLabel::setVisible);
}

void ReplayWindow::LoadHistory(std::string fpath, bool debugMode)
{
	fullHistory = Load(fpath);
	FillHistoryTable();
	withDelay->setChecked(!debugMode);
	if (debugMode)
	{
		// Start immediately
		playPauseButton->setChecked(true);
	}
}

void ReplayWindow::closeEvent(QCloseEvent* e)
{
	playPauseButton->setChecked(false);
	QDialog::closeEvent(e);
}

std::vector<RoadRunner::UserAction> ReplayWindow::Load(std::string fpath)
{
	std::ifstream inFile(fpath, std::ios::binary);
	cereal::BinaryInputArchive iarchive(inFile);
	std::vector<RoadRunner::UserAction> rtn;
	iarchive(rtn);
	return rtn;
}

void ReplayWindow::FillHistoryTable()
{
	listWidget->clear();
	bool supported = true;
	for (const auto& action : fullHistory)
	{
		QString desc;
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
			case Qt::MouseButton::MiddleButton:
				desc += " mid";
				break;
			default:
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
				case Qt::MouseButton::MiddleButton:
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
				icon = QIcon(QPixmap(":/icons/Mouse.png"));
				break;
			default:
				supported = false;
				break;
			}
			desc += QString(" (%1, %2)")
				.arg(action.detail.mouse.sceneX, 4, 'f', 2)
				.arg(action.detail.mouse.sceneY, 4, 'f', 2);
			break;
		case RoadRunner::ActionType::Action_KeyPress:
			desc = QString("KeyPress %1").arg(action.detail.keyPress.ToString());
			icon = QIcon(QPixmap(":/icons/keyboard.png"));
			break;
		case RoadRunner::ActionType::Action_Viewport:
			desc = QString("ViewChange zoom:%1 rot: %2")
				.arg(action.detail.viewport.zoom)
				.arg(action.detail.viewport.rotate);
			icon = QIcon(QPixmap(":/icons/zoomin.png"));
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
			case RoadRunner::Mode_Create:
				desc += " road";
				icon = QIcon(QPixmap(":/icons/road_mode.png"));
				break;
			case RoadRunner::Mode_CreateLanes:
				desc += " lane";
				icon = QIcon(QPixmap(":/icons/lane_mode.png"));
				break;
			case RoadRunner::Mode_Modify:
				desc += " modify";
				icon = QIcon(QPixmap(":/icons/modify_mode.PNG"));
				break;
			case RoadRunner::Mode_Destroy:
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
			desc = QString("ChangeProfile Lane:%1 Offset:%2 | Lane:%3 Offset:%4")
				.arg(action.detail.changeProfile.leftProfile.laneCount)
				.arg(action.detail.changeProfile.leftProfile.offsetx2)
				.arg(action.detail.changeProfile.rightProfile.offsetx2)
				.arg(action.detail.changeProfile.rightProfile.offsetx2);
			icon = QIcon(QPixmap(":/icons/car_coming.png"));
			break;
		case RoadRunner::ActionType::Action_ResizeWindow:
			desc = QString("Resize windows from (%1,%2) to (%3,%4)")
				.arg(action.detail.resizeWindow.oldWidth)
				.arg(action.detail.resizeWindow.oldHeight)
				.arg(action.detail.resizeWindow.width)
				.arg(action.detail.resizeWindow.height);
			icon = QIcon(QPixmap(":/icons/resize.png"));
			break;
		default:
			supported = false;
			break;
		} 

		auto item = new QListWidgetItem(icon, desc, listWidget);
		item->setToolTip("Dbl-click to toggle breakpoint");
		listWidget->addItem(item);
	}

	nextToReplay = 0;
	if (!fullHistory.empty())
	{
		listWidget->item(nextToReplay)->setForeground(NextReplayFG);
	}
	if (!supported)
	{
		spdlog::error("Found unsupported action type. History might be corrupted file or from future version!");
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
		item->setBackground(QBrush(Qt::magenta));
		item->setData(BreakPointFlag, true);
	}
}

void ReplayWindow::SingleStep()
{
	if (nextToReplay >= fullHistory.size())
	{
		return;
	}

	if (!withDelay->isChecked())
	{
		// Skip intermediate mouse moves in debug mode to save time
		while (nextToReplay < fullHistory.size() - 1)
		{
			bool consecutiveMouseMove = 
				fullHistory[nextToReplay].type == RoadRunner::Action_Mouse
				&& fullHistory[nextToReplay].detail.mouse.type == QEvent::MouseMove
				&& fullHistory[nextToReplay + 1].type == RoadRunner::Action_Mouse
				&& fullHistory[nextToReplay + 1].detail.mouse.type == QEvent::MouseMove;
			bool consecutiveViewchange =
				fullHistory[nextToReplay].type == RoadRunner::Action_Viewport
				&& fullHistory[nextToReplay + 1].type == RoadRunner::Action_Viewport;
			if ((consecutiveMouseMove || consecutiveViewchange)
				&& !listWidget->item(nextToReplay)->data(BreakPointFlag).toBool())
			{
				auto skipItem = listWidget->item(nextToReplay);
				skipItem->setForeground(Qt::NoBrush);
				skipItem->setFlags(skipItem->flags() & ~Qt::ItemIsEnabled);
				nextToReplay++;
			}
			else
			{
				break;
			}
		}
	}

	auto replayingItem = listWidget->item(nextToReplay);
	replayingItem->setForeground(Qt::NoBrush);
	replayingItem->setFlags(replayingItem->flags() & ~Qt::ItemIsEnabled);
	if (replayingItem->data(BreakPointFlag).toBool())
	{
		// break point triggered: pause
		if (playPauseButton->isChecked())
		{
			playPauseButton->setChecked(false);
		}
	}

	if (withDelay->isChecked())
	{
		const auto& action = fullHistory[nextToReplay];
		if (action.type == RoadRunner::Action_Mouse)
		{
			mouseStatus->setPixmap(replayingItem->icon().pixmap(replayingItem->icon().availableSizes().first()));
		}

		QImage kbImage(":/icons/keyboard.png");
		if (action.type == RoadRunner::Action_KeyPress)
		{
			QPainter* p = new QPainter(&kbImage);
			p->setPen(Qt::black);
			p->setFont(QFont("Arial", 10));

			p->drawText(kbImage.rect(), Qt::AlignHCenter, action.detail.keyPress.ToString());
		}
		keyStatus->setPixmap(QPixmap::fromImage(kbImage));
	}

	RoadRunner::ActionManager::Instance()->Replay(fullHistory[nextToReplay++]);
	if (nextToReplay < fullHistory.size())
	{
		listWidget->item(nextToReplay)->setForeground(NextReplayFG);
		listWidget->scrollTo(listWidget->model()->index(nextToReplay, 0));
	}
	else
	{
		playPauseButton->setChecked(false);
		emit DoneReplay(true);
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
	if (pausedMS >= requiredDelay || pausedMS >= MaximumPauseMS)
	{
		SingleStep();
		pausedMS = 0;
	}
}

void ReplayWindow::PlayPause(bool play)
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
	}
	nextToReplay = 0;
	if (!fullHistory.empty())
	{
		listWidget->item(nextToReplay)->setForeground(NextReplayFG);
	}

	emit Restart();
}