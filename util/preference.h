#pragma once

#include <string>
#include <QDialog>

struct UserPreference
{
    bool showWelcome;
    bool alwaysVerify;
    bool antiAlias;

    template<class Archive>
    void serialize(Archive& archive)
    {
        archive(showWelcome, alwaysVerify, antiAlias);
    }
};

class PreferenceWindow: public QDialog
{
Q_OBJECT
public:
    PreferenceWindow(QWidget* parent);

signals:
    void ToggleAA(bool);

protected:
    void showEvent(QShowEvent*) override;

    void closeEvent(QCloseEvent*) override;

private:
    /*Lazy-populate to save resource during startup*/
    bool contentPopulated;
};

extern UserPreference g_preference;
