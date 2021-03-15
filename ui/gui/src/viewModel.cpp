#include "gui/viewModel.h"
#include "ui_mainWindow.h"

#include "gui/preferencesDialog.h"
#include "model/habiTrack.h"

#include <spdlog/spdlog.h>


using namespace ht;

namespace gui
{
ViewModel::ViewModel(QObject* parent, Ui::MainWindow* ui)
    : QObject(parent)
    , mUi(ui)
    , mHabiTrack(std::make_unique<HabiTrack>())
{
}

ViewModel::~ViewModel() { }

void ViewModel::setupConnections()
{
    setupActions();
}

void ViewModel::setupActions()
{
    connect(mUi->actionPreferences, SIGNAL(triggered()), this, SLOT(actionPreferencesTriggered()));
}

void ViewModel::actionPreferencesTriggered()
{
    auto oldPrefs = mHabiTrack->getPreferences();
    PreferencesDialog prefDialog(this, oldPrefs);
    auto code = prefDialog.exec();
    if (code == QDialog::Accepted)
    {
        // overwrite current settings if accepted, discard otherwise
        auto newPrefs = prefDialog.getPreferences();

        if (newPrefs != oldPrefs)
        {
            spdlog::debug("GUI: Changed Preferences to: {}", newPrefs);
        }

        // TODO: should this be saved implicitly?
        /* saveResults(mResultsFile, mPrefs, mImgFolder, mStartFrameNumber, mEndFrameNumber); */
    }
}

} // namespace gui


