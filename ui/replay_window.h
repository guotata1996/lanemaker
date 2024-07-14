#pragma once

#include <QDialog>
#include <vector>

QT_BEGIN_NAMESPACE
class QListWidget;
class QListWidgetItem;
class QTimer;
class QPushButton;
class QCheckBox;
class QLabel;
QT_END_NAMESPACE

namespace RoadRunner
{
	struct UserAction;
}

class ReplayWindow : public QDialog
{
Q_OBJECT

public:
	ReplayWindow(QWidget* parent = nullptr);

	void LoadHistory(std::string fpath, bool startImmediate=false);	

signals:
	void Restart();

	void DoneReplay(bool completed);

private slots:
	void PlaceBreakpointOn(QListWidgetItem*);

	void PlayPause(bool play);

	void SingleStep();

	void StepRealDelay();

	void ReplayFromStart();

protected:
	void closeEvent(QCloseEvent* e) override;

private:
	void FillHistoryTable();

	QListWidget* listWidget;
	QLabel* mouseStatus;
	QLabel* keyStatus;
	QTimer* replayTimer;
	QPushButton* resetButton;
	QPushButton* playPauseButton;
	QCheckBox* withDelay;
	std::vector<RoadRunner::UserAction> fullHistory;
	size_t nextToReplay;
	bool supported;

	const Qt::GlobalColor NextReplayFG = Qt::darkMagenta;

	const int BreakPointFlag = Qt::UserRole + 100;

	const int TimerInterval = 50;
	int pausedMS;
};