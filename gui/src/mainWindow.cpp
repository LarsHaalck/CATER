#include "gui/mainWindow.h"
#include "ui_mainWindow.h"

#include "gui/preferencesDialog.h"

#include <iostream>


namespace gui
{
HabiTrack::HabiTrack(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::HabiTrack)
    , mGuiPrefs()
    , mGuiPrefsDefaults()
    , mPrefs()
{
    ui->setupUi(this);
    populateGuiDefaults();


    //////////////////////////////////////////////////////////////////////////////////
    // OVERLAY SLIDER CONNECTS AND FILTERS (no need to pass them to controller)
    //////////////////////////////////////////////////////////////////////////////////
    connect(ui->sliderOverlayUnaries, SIGNAL(valueChanged(int)), this,
        SLOT(onSliderOverlayUnariesChanged(int)));
    connect(ui->sliderOverlayTrackedPos, SIGNAL(valueChanged(int)), this,
        SLOT(onSliderOverlayTrackedPosChanged(int)));
    connect(ui->sliderOverlayTrajectory, SIGNAL(valueChanged(int)), this,
        SLOT(onSliderOverlayTrajectoryChanged(int)));

    ui->sliderOverlayUnaries->installEventFilter(this);
    ui->sliderOverlayTrackedPos->installEventFilter(this);
    ui->sliderOverlayTrajectory->installEventFilter(this);

    connect(ui->actionExpertMode, SIGNAL(toggled(bool)), this,
        SLOT(onToggleExpertMode(bool)));

    connect(ui->actionPreferences, SIGNAL(triggered()), this,
        SLOT(onPreferencesClicked()));

}

HabiTrack::~HabiTrack() { delete ui; }


void HabiTrack::populateGuiDefaults()
{
    // set sliders
    resetToDefaults(ui->sliderOverlayUnaries);
    resetToDefaults(ui->sliderOverlayTrackedPos);
    resetToDefaults(ui->sliderOverlayTrajectory);

    // set buttons
    resetToDefaults(ui->actionExpertMode);
}

bool HabiTrack::eventFilter(QObject* obj, QEvent*  event)
{
    if (event->type() == QEvent::MouseButtonRelease)
    {
        // static_cast is safe because it is a QMouseEvent
        auto mouseEvent = static_cast<QMouseEvent*>(event);
        if (mouseEvent->button() == Qt::RightButton
            && mouseEvent->modifiers() == Qt::ControlModifier)
        {
            resetToDefaults(obj);
            return true;
        }
    }
    return false;
}

void HabiTrack::resetToDefaults(QObject* obj)
{
    if (obj == ui->sliderOverlayUnaries)
        ui->sliderOverlayUnaries->setValue(mGuiPrefsDefaults.overlayUnaries);

    if (obj == ui->sliderOverlayTrackedPos)
        ui->sliderOverlayTrackedPos->setValue(mGuiPrefsDefaults.overlayTrackedPos);

    if (obj == ui->sliderOverlayTrajectory)
        ui->sliderOverlayTrajectory->setValue(mGuiPrefsDefaults.overlayTrajectory);

    if (obj == ui->actionExpertMode)
    {
        ui->actionExpertMode->setChecked(mGuiPrefsDefaults.enableExpertView);
        onToggleExpertMode(mGuiPrefsDefaults.enableExpertView);
    }

}

//////////////////////////////////////////////////////////////////////
// OVERLAY SLIDERS
//////////////////////////////////////////////////////////////////////
void HabiTrack::onSliderOverlayUnariesChanged(int value)
{
    mGuiPrefs.overlayUnaries = value;
}

void HabiTrack::onSliderOverlayTrackedPosChanged(int value)
{
    mGuiPrefs.overlayTrackedPos = value;
}

void HabiTrack::onSliderOverlayTrajectoryChanged(int value)
{
    mGuiPrefs.overlayTrajectory = value;
}

void HabiTrack::onToggleExpertMode(bool value)
{
    ui->frameTracking->setVisible(value);
    // implicitly hidden by hiding the frame
    /* ui->buttonExtractFeatures->setVisible(value); */
    /* ui->buttonExtractTrafos->setVisible(value); */
    /* ui->buttonExtractUnaries->setVisible(value); */
    /* ui->buttonOptimizeUnaries->setVisible(value); */

    ui->framePano->setVisible(value);
    // implicitly hidden by hiding the frame
    /* ui->buttonKeyFrameSelection->setVisible(value); */
    /* ui->buttonOptimizeKeyFrames->setVisible(value); */
    /* ui->buttonReintegrateFrames->setVisible(value); */
    /* ui->buttonOptimizeAllFrames->setVisible(value); */

    ui->labelExpertMode->setVisible(value);
}

void HabiTrack::onPreferencesClicked()
{
    PreferencesDialog prefDialog(this, mPrefs);
    auto code = prefDialog.exec();
    if (code == QDialog::Accepted)
    {
        // overwrite current settings if accepted, discard otherwise
        mPrefs = prefDialog.getPreferences();
        std::cout << mPrefs << std::endl;
    }

}


}
