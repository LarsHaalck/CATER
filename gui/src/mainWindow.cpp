#include "gui/mainWindow.h"

#include "gui/preferencesDialog.h"
#include "gui/progressStatusBar.h"
#include "gui/qtOpencvCore.h"
#include "ui_mainWindow.h"
#include <QDate>
#include <QFileDialog>
#include <QMessageBox>
#include <QTime>
#include <chrono>
#include <iostream>
#include <spdlog/spdlog.h>

#include "resultsIO.h"

using namespace ht;
namespace fs = std::filesystem;

namespace gui
{
HabiTrack::HabiTrack(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::HabiTrack)
    , mStartPath()
    , mBar(nullptr)
    , mGuiPrefs()
    , mGuiPrefsDefaults()
    , mPrefs()
    , mImages()
    , mCurrentFrameNumber(0)
    , mScene(nullptr)
    , mFeatures()
    , mUnaries()
{
    ui->setupUi(this);
    populateGuiDefaults();
    mScene = ui->graphicsView->getTrackerScene();
    mBar = std::make_shared<ProgressStatusBar>(ui->progressBar, ui->labelProgress);

    // install filters for "resettable" sliders
    ui->sliderOverlayUnaries->installEventFilter(this);
    ui->sliderOverlayTrackedPos->installEventFilter(this);
    ui->sliderOverlayTrajectory->installEventFilter(this);
}

HabiTrack::~HabiTrack() { delete ui; }

void HabiTrack::populateGuiDefaults()
{
    spdlog::debug("GUI: Populating GUI defaults");

    // set sliders
    resetToDefaults(ui->sliderOverlayUnaries);
    resetToDefaults(ui->sliderOverlayTrackedPos);
    resetToDefaults(ui->sliderOverlayTrajectory);

    // set buttons
    resetToDefaults(ui->actionExpertMode);
}

bool HabiTrack::eventFilter(QObject* obj, QEvent* event)
{
    if (event->type() == QEvent::MouseButtonRelease)
    {
        // static_cast is safe because it is a QMouseEvent
        auto mouseEvent = static_cast<QMouseEvent*>(event);
        if (mouseEvent->button() == Qt::RightButton
            && mouseEvent->modifiers() == Qt::ControlModifier)
        {
            spdlog::debug("GUI: Captured event filter to reset gui element");
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
        ui->actionExpertMode->setChecked(mGuiPrefsDefaults.enableExpertView);
}

//////////////////////////////////////////////////////////////////////
// SLOTS
//////////////////////////////////////////////////////////////////////
void HabiTrack::on_sliderOverlayUnaries_sliderReleased()
{
    spdlog::debug("GUI: Slider overlay unaries changed");
    auto value = ui->sliderOverlayUnaries->value();
    mGuiPrefs.overlayUnaries = value;
    showFrame(mCurrentFrameNumber);
}

void HabiTrack::on_sliderOverlayTrackedPos_sliderReleased()
{
    spdlog::debug("GUI: Slider overlay tracked pos changed");
    auto value = ui->sliderOverlayTrackedPos->value();
    mGuiPrefs.overlayTrackedPos = value;
    showFrame(mCurrentFrameNumber);
}

void HabiTrack::on_sliderOverlayTrajectory_sliderReleased()
{
    spdlog::debug("GUI: Slider overlay trajectory changed");
    auto value = ui->sliderOverlayTrajectory->value();
    mGuiPrefs.overlayTrajectory = value;
    showFrame(mCurrentFrameNumber);
}

void HabiTrack::on_actionExpertMode_toggled(bool value)
{
    spdlog::debug("GUI: Toggled expert mode");
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

void HabiTrack::on_actionPreferences_triggered()
{
    PreferencesDialog prefDialog(this, mPrefs);
    auto code = prefDialog.exec();
    if (code == QDialog::Accepted)
    {
        // overwrite current settings if accepted, discard otherwise
        mPrefs = prefDialog.getPreferences();
        spdlog::debug("GUI: Changed Preferences to: {}", mPrefs);
        saveResults(mOutputPath / "results.yml", mPrefs);
    }
}

void HabiTrack::on_sliderFrame_sliderReleased()
{
    spdlog::debug("GUI: Changed slider frame");
    int value = ui->sliderFrame->value();
    showFrame(value);
}

void HabiTrack::on_spinCurrentFrame_editingFinished()
{
    spdlog::debug("GUI: Changed frame spin (edited)");
    int value = ui->spinCurrentFrame->value();
    showFrame(value);
}

void HabiTrack::on_spinCurrentFrame_valueChanged(int value)
{
    spdlog::debug("GUI: Changed frame spin");
    showFrame(value);
}

void HabiTrack::on_buttonPrevFrame_clicked()
{
    spdlog::debug("GUI: prev clicked");
    if (mCurrentFrameNumber > 1)
    {
        auto newIdx = mCurrentFrameNumber - 1;
        showFrame(newIdx);
    }
}

void HabiTrack::on_buttonNextFrame_clicked()
{
    spdlog::debug("GUI: next clicked");
    if (mCurrentFrameNumber < mImages.size())
    {
        auto newIdx = mCurrentFrameNumber + 1;
        showFrame(newIdx);
    }
}

void HabiTrack::on_actionPrev_Frame_triggered() { on_buttonPrevFrame_clicked(); }

void HabiTrack::on_actionNext_Frame_triggered() { on_buttonNextFrame_clicked(); }

void HabiTrack::refreshWindow()
{
    spdlog::debug("GUI: Redraw window");

    ui->sliderFrame->blockSignals(true);
    ui->spinCurrentFrame->blockSignals(true);

    ui->sliderFrame->setValue(mCurrentFrameNumber);
    ui->spinCurrentFrame->setValue(mCurrentFrameNumber);

    ui->sliderFrame->blockSignals(false);
    ui->spinCurrentFrame->blockSignals(false);

    qApp->processEvents();
}

void HabiTrack::populatePaths(const fs::path& path)
{
    if (fs::is_directory(path))
        mOutputPath = path;
    else
        mOutputPath = path.filename();

    mOutputPath += "_output";

    auto date = QDate::currentDate().toString("yyyy-MM-dd").toStdString();
    auto time = QTime::currentTime().toString("hh-mm-ss").toStdString();

    mOutputPath /= "now";
    /* mOutputPath /= date + "_" + time; */

    spdlog::debug("Generated output path: {}", mOutputPath.string());

    mResultsFile = mOutputPath / "results.yml";
    mFtFolder = mOutputPath / "fts";
    mMatchFolder = mOutputPath / "matches";
    mUnFolder = mOutputPath / "unaries";
    mAntFile = mOutputPath / "ant.yml";
}

void HabiTrack::showFrame(std::size_t frameNumber)
{
    spdlog::debug("GUI: Show frame {}", frameNumber);
    mCurrentFrameNumber = frameNumber;
    cv::Mat frame = mImages.at(frameNumber - 1); // zero indexed here

    // set filename
    ui->labelFileName->setText(
        QString::fromStdString(mImages.getFileName(frameNumber - 1).string()));

    // overlay unary
    int unarySlider = ui->sliderOverlayUnaries->value();
    if (unarySlider > 0 && mUnaries.exists(mCurrentFrameNumber - 1))
    {
        double alpha = static_cast<double>(unarySlider) / 100.0;
        auto unary = mUnaries.at(mCurrentFrameNumber - 1);

        // todo multiply with gaussian mask from tracker
        /* auto mask */

        cv::Mat unaryColor;
        cv::cvtColor(unary, unaryColor, cv::COLOR_GRAY2BGR);
        std::vector<cv::Mat> channels(3);
        cv::split(unaryColor, channels);
        channels[0] = cv::Mat::zeros(unary.size(), CV_8UC1);
        cv::merge(channels, unaryColor);

        cv::resize(unaryColor, unaryColor, frame.size());

        cv::addWeighted(unaryColor, alpha, frame, (1 - alpha), 0.0, frame);
    }
    /* if (unarySlider > 0 */
    /*     && mTrackingData.getPreviewUnaryAt(mCurrentFrameNumber, unary)) */
    /* { */
    /*     double alpha = static_cast<double>(unarySlider) / 100.0; */

    /*     std::vector<cv::Mat> channels(3); */
    /*     cv::split(unary, channels); */
    /*     channels[0] = cv::Mat::zeros(unary.size(), CV_8UC1); */
    /*     cv::merge(channels, unary); */
    /*     cv::resize(unary, unary, mCurrentFrame.size()); */

    /*     cv::addWeighted( */
    /*         unary, alpha, mCurrentFrame, (1 - alpha), 0.0, mCurrentFrame); */
    /* } */

    /* // get ant position */
    /* int antPositionSlider = ui->horizontalSlider_ant_positions->value(); */
    /* cv::Point antPosition; */
    /* if (antPositionSlider > 0 */
    /*     && mTrackingData.getAntPositionAt(mCurrentFrameNumber, antPosition)) */
    /* { */
    /*     double theta; */
    /*     mTrackingData.getAntThetaAt(mCurrentFrameNumber, theta); */

    /*     double thetaQuality; */
    /*     mTrackingData.getAntThetaQualityAt(mCurrentFrameNumber, thetaQuality); */

    /*     cv::Point dirIndicator = utils::rotatePointAroundPoint(antPosition, theta); */

    /*     int red = antPositionSlider * 255 / 100; */
    /*     int blue = antPositionSlider * 100 / 100; */
    /*     int green = antPositionSlider * 100 / 100; */
    /*     int circleThickness = 1; */
    /*     if (antPositionSlider == 100) */
    /*     { */
    /*         circleThickness = 2; */
    /*         cv::line(mCurrentFrame, antPosition, dirIndicator, */
    /*             cv::Scalar(0, 255, 0), 1); */
    /*     } */
    /*     cv::Scalar color(blue, green, red); */
    /*     cv::circle(mCurrentFrame, antPosition, 20, color, circleThickness); */
    /* } */

    /* // get trajectory */
    /* int trajectorySlider = ui->horizontalSlide_draw_trajectory->value(); */
    /* std::vector<cv::Point> track; */
    /* if (trajectorySlider > 0 */
    /*     && mTrackingData.getAntTrajectoryAt( */
    /*            mCurrentFrameNumber, trajectorySlider, track)) */
    /* { */
    /*     for (unsigned int i = 1; i < track.size(); ++i) */
    /*     { */
    /*         cv::Point pre = track.at(i - 1); */
    /*         cv::Point cur = track.at(i); */
    /*         cv::line(mCurrentFrame, pre, cur, cv::Scalar(0, 255, 255), 2); */
    /*     } */
    /* } */

    // set combined image and redraw
    auto pixMap = QPixmap::fromImage(QtOpencvCore::img2qimgRaw(frame));
    mScene->setPixmap(pixMap);
    refreshWindow();
}

void HabiTrack::openImagesHelper()
{
    auto numImgs = mImages.size();

    // set up gui elements
    ui->labelMaxFrames->setText(QString::number(numImgs));
    ui->sliderFrame->setMinimum(1);
    ui->sliderFrame->setMaximum(numImgs);
    ui->sliderFrame->setValue(1);
    ui->sliderFrame->setEnabled(true);

    ui->spinCurrentFrame->setMinimum(1);
    ui->spinCurrentFrame->setMaximum(numImgs);
    ui->spinCurrentFrame->setValue(1);
    ui->spinCurrentFrame->setEnabled(true);

    ui->labelFileName->setText(QString::fromStdString(mImages.getFileName(0).string()));
    ui->labelStartFrame->setText(QString::number(1));
    ui->labelEndFrame->setText(QString::number(numImgs));
    mStartFrameNumber = 1;
    mEndFrameNumber = numImgs;

    ui->labelNumTrafos->setText(QString::number(0));
    ui->labelNumUnaries->setText(QString::number(0));
    ui->labelNumTrackedPos->setText(QString::number(0));

    ui->buttonPrevFrame->setEnabled(true);
    ui->buttonNextFrame->setEnabled(true);
    ui->actionPrev_Frame->setEnabled(true);
    ui->actionNext_Frame->setEnabled(true);

    ui->buttonTrack->setEnabled(true);
    ui->frameTracking->setEnabled(true);
    for (auto& child : ui->frameTracking->findChildren<QWidget*>())
        child->setEnabled(true);

    ui->buttonStartFrame->setEnabled(true);
    ui->buttonEndFrame->setEnabled(true);

    // show first frame
    mCurrentFrameNumber = 1;
    showFrame(1);

    populatePaths(fs::path(mStartPath.toStdString()));
}

void HabiTrack::on_actionOpenImgFolder_triggered()
{
    spdlog::debug("GUI: Triggered Open image folder");
    QString imgFolderPath = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
        mStartPath, QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (imgFolderPath.isEmpty())
        return;
    mStartPath = imgFolderPath;

    mImgFolder = fs::path(imgFolderPath.toStdString());
    mImages = Images(mImgFolder);
    openImagesHelper();
}

void HabiTrack::on_actionOpenImgListtriggered()
{
    spdlog::debug("GUI: Triggered open image list file");
    QString imgFilePath = QFileDialog::getOpenFileName(this, tr("Open Image File List"),
        mStartPath, "Text (*.txt)");
    if (imgFilePath.isEmpty())
        return;
    mStartPath = imgFilePath;

    mImgFolder = fs::path(imgFilePath.toStdString());
    mImages = Images(mImgFolder);
    openImagesHelper();
}

void HabiTrack::on_actionOpenResultsFile_triggered()
{
}

void HabiTrack::on_buttonStartFrame_clicked()
{
    if (mCurrentFrameNumber >= mEndFrameNumber)
    {
        QMessageBox::warning(
            this, "Warning", "Start frame cannot be equal or greater than end frame");
        return;
    }
    spdlog::debug("GUI: changed start frame to {}", mCurrentFrameNumber);
    // TODO: some data structures are no longer valid and should be computed again
    mStartFrameNumber = mCurrentFrameNumber;
    ui->labelStartFrame->setText(QString::number(mStartFrameNumber));
}

void HabiTrack::on_buttonEndFrame_clicked()
{
    if (mCurrentFrameNumber <= mStartFrameNumber)
    {
        QMessageBox::warning(
            this, "Warning", "End frame cannot be equal or smaller than start frame");
        return;
    }
    spdlog::debug("GUI: changed end frame to {}", mCurrentFrameNumber);
    // TODO: some data structures are no longer valid and should be computed again
    mEndFrameNumber = mCurrentFrameNumber;
    ui->labelEndFrame->setText(QString::number(mEndFrameNumber));
}

void HabiTrack::on_buttonExtractFeatures_clicked()
{
    spdlog::debug("GUI: Clicked Extract Features");

    auto start = mStartFrameNumber - 1;
    auto end = mEndFrameNumber;
    if (Features::isComputed(mImages, mFtFolder, mPrefs.featureType, start, end))
    {
        mFeatures = Features::fromDir(mImages, mFtFolder, mPrefs.featureType, start, end);
        mBar->done();
        return;
    }
    mFeatures = Features::compute(mImages, mFtFolder, mPrefs.featureType, mPrefs.numFeatures, start,
        end, mPrefs.cacheSize, mBar);
}

void HabiTrack::on_buttonExtractTrafos_clicked()
{
    auto start = mStartFrameNumber - 1;
    auto end = mEndFrameNumber;
    if (!Features::isComputed(mImages, mFtFolder, mPrefs.featureType, start, end))
    {
        QMessageBox::warning(this, "Warning", "Features need to be computed first");
        return;
    }


    spdlog::debug("GUI: Clicked Extract Trafos");
    if (matches::isComputed(mMatchFolder, GeometricType::Homography))
    {
        mBar->done();
        return;
    }
    matches::compute(mMatchFolder, GeometricType::Homography, mFeatures,
        matches::MatchType::Windowed, 2, 0.0, nullptr, mPrefs.cacheSize, size_t_vec(), mBar);
}

void HabiTrack::on_buttonExtractUnaries_clicked()
{
    spdlog::debug("GUI: Clicked Extract Unaries");
    if (!matches::isComputed(mMatchFolder, GeometricType::Homography))
    {
        QMessageBox::warning(this, "Warning", "Transformations need to be computed first");
        return;
    }

    auto start = mStartFrameNumber - 1;
    auto end = mEndFrameNumber;
    if (Unaries::isComputed(mImgFolder, mUnFolder, start, end))
    {
        mUnaries = Unaries::fromDir(mImgFolder, mUnFolder, start, end);
        mBar->done();
        return;
    }
    auto trafos = mPrefs.removeCamMotion
        ? matches::getTrafos(mMatchFolder, GeometricType::Homography)
        : matches::PairwiseTrafos();

    mUnaries = Unaries::compute(mImgFolder, mUnFolder, start, end, mPrefs.removeRedLasers,
        mPrefs.unarySubsample, trafos, mPrefs.cacheSize, mBar);

    ui->unaryView->getUnaryScene()->setTotalImages(mImages.size());
}
} // namespace gui
