#pragma once

#include <QDialog>
#include <vector>

QT_BEGIN_NAMESPACE
class QListWidget;
class QListWidgetItem;
class QTimer;
class QPushButton;
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

	void LoadHistory(std::string fpath);

signals:
	void Restart();

private slots:
	/*RoadRunnerTODO: merge play immediate into window and support break point*/
	void PlaceBreakpointOn(QListWidgetItem*);

	void ToggleAnimatedPlay(bool play);

	void SingleStep();

	void ReplayFromStart();

private:
	void FillHistoryTable();

	QListWidget* listWidget;
	QTimer* replayTimer;
	QPushButton* resetButton;
	std::vector<RoadRunner::UserAction> fullHistory;
	size_t nextToReplay;
	bool supported;

	const int BreakPointFlag = Qt::UserRole + 100;
};