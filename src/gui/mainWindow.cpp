#include "gui/mainWindow.h"
#include "ui_mainWindow.h"

#include <iostream>


namespace gui
{
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , mGuiPrefs()
{
    ui->setupUi(this);
    populateGuiDefaults();


    ////////////////////////////////////////////////////
    // OVERLAY SLIDER CONNECTS
    ////////////////////////////////////////////////////
    connect(ui->sliderOverlayUnaries, SIGNAL(valueChanged(int)), this, 
        SLOT(onSliderOverlayUnariesChanged(int)));
    connect(ui->sliderOverlayTrackedPos, SIGNAL(valueChanged(int)), this, 
        SLOT(onSliderOverlayTrackedPosChanged(int)));
    connect(ui->sliderOverlayTrajectory, SIGNAL(valueChanged(int)), this, 
        SLOT(onSliderOverlayTrajectoryChanged(int)));
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::populateGuiDefaults()
{
    ui->sliderOverlayUnaries->setValue(mGuiPrefs.overlayUnaries);
    ui->sliderOverlayTrackedPos->setValue(mGuiPrefs.overlayTrackedPos);
    ui->sliderOverlayTrajectory->setValue(mGuiPrefs.overlayTrajectory);

    if (!mGuiPrefs.enableExpertView)
    {
        ui->buttonExtractFeatures->setVisible(false);
        ui->buttonExtractTrafos->setVisible(false);
        ui->buttonExtractUnaries->setVisible(false);
        ui->buttonKeyFrameSelection->setVisible(false);
        ui->buttonReintegrateFrames->setVisible(false);
    }
}

//////////////////////////////////////////////////////////////////////
// OVERLAY SLIDERS
//////////////////////////////////////////////////////////////////////
void MainWindow::onSliderOverlayUnariesChanged(int value)
{
    mGuiPrefs.overlayUnaries = value;
}

void MainWindow::onSliderOverlayTrackedPosChanged(int value)
{
    mGuiPrefs.overlayTrackedPos = value;
}

void MainWindow::onSliderOverlayTrajectoryChanged(int value)
{
    mGuiPrefs.overlayTrajectory = value;
}
}
