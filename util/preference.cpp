#include "preference.h"
#include "util.h"

#include <QtWidgets>

#include <filesystem>
#include <fstream>
#include <cereal/types/string.hpp>
#include <cereal/archives/json.hpp>

#include <spdlog/spdlog.h>


UserPreference g_preference;

PreferenceWindow::PreferenceWindow(QWidget* parent)
    : QDialog(parent), contentPopulated(false)
{
    auto defaultSave = RoadRunner::DefaultSaveFolder() / "preference.json";
    if (std::filesystem::exists(defaultSave))
    {
        std::ifstream inFile(defaultSave);
        cereal::JSONInputArchive iarchive(inFile);
        iarchive(g_preference);
    }
    else
    {
        g_preference.showWelcome = true;
        g_preference.alwaysVerify = false;
        g_preference.antiAlias = true;
    }
}

void PreferenceWindow::showEvent(QShowEvent* e)
{
    if (!contentPopulated)
    {
        contentPopulated = true;
        QGridLayout* layout = new QGridLayout(this);
        layout->addWidget(new QLabel("Show welcome"), 0, 0);
        auto showWelcomeChoice = new QCheckBox;
        showWelcomeChoice->setChecked(g_preference.showWelcome);
        layout->addWidget(showWelcomeChoice, 0, 1);

        layout->addWidget(new QLabel("Always verify"), 1, 0);
        auto alwaysVerifyChoice = new QCheckBox;
        alwaysVerifyChoice->setChecked(g_preference.alwaysVerify);
        layout->addWidget(alwaysVerifyChoice, 1, 1);

        layout->addWidget(new QLabel("Anti alias"), 2, 0);
        auto antiAliasChoice = new QCheckBox;
        antiAliasChoice->setChecked(g_preference.antiAlias);
        layout->addWidget(antiAliasChoice, 2, 1);

        setLayout(layout);

        connect(showWelcomeChoice, &QCheckBox::toggled,
            [](bool c) {g_preference.showWelcome = c; });
        connect(alwaysVerifyChoice, &QCheckBox::toggled,
            [](bool c) {g_preference.alwaysVerify = c; });
        connect(antiAliasChoice, &QCheckBox::toggled,
            [this](bool c) {
                g_preference.antiAlias = c;
                emit ToggleAA(c); });
    }

    QDialog::showEvent(e);
}

void PreferenceWindow::closeEvent(QCloseEvent* e)
{
    auto defaultSave = RoadRunner::DefaultSaveFolder() / "preference.json";
    std::ofstream outFile(defaultSave);
    cereal::JSONOutputArchive oarchive(outFile);
    oarchive(g_preference);

    QDialog::closeEvent(e);
}
