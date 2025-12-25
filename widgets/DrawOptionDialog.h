#include "AnimatedPopupDialog.h"

#include <QSlider>
#include <QDial>
#include <QLabel>

class LaneConfigWidget;

class DrawOptionDialog : public AnimatedPopupDialog
{
    Q_OBJECT
public:
    DrawOptionDialog(QWidget* parent);

protected:
    void showEvent(QShowEvent*) override;
    void closeEvent(QCloseEvent*) override;

private:
    LaneConfigWidget* laneConfig;
    QDial* heightConfig;
    QLabel* heightDisplay;
};