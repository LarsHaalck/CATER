#include "gui/mainWindow.h"
#include "ui_mainWindow.h"

#include "gui/preferencesDialog.h"
#include "gui/progressStatusBar.h"
#include "gui/qtOpencvCore.h"
#include "habitrack/tracker.h"
#include "resultsIO.h"
#include <QDate>
#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <QTime>
#include <QtConcurrent>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <spdlog/spdlog.h>
#include "image-processing/util.h"


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
    , mPrefs()
    , mImages()
    , mCurrentFrameNumber(0)
    , mScene(nullptr)
    , mFeatures()
    , mUnaries()
    , mManualUnaries()
    , mDetections()
    , mDetectionsWatchers()
    , mDetectionsQueue()
{
    ui->setupUi(this);
    populateGuiDefaults();

    // needs to be done after setupUi
    mScene = ui->graphicsView->getTrackerScene();
    mBar = std::make_shared<ProgressStatusBar>(ui->progressBar, ui->labelProgress);

    connect(ui->unaryView, &UnaryGraphicsView::jumpedToUnary, this,
        [=](std::size_t num) { this->showFrame(num + 1); });

    connect(this->ui->graphicsView, SIGNAL(positionChanged(QPointF)), this,
        SLOT(onPositionChanged(QPointF)));
    connect(this->ui->graphicsView, SIGNAL(bearingChanged(QPointF)), this,
        SLOT(onBearingChanged(QPointF)));
    connect(this->ui->graphicsView, SIGNAL(positionCleared()), this, SLOT(onPositionCleared()));
    connect(this->ui->graphicsView, SIGNAL(bearingCleared()), this, SLOT(onBearingCleared()));
}

HabiTrack::~HabiTrack() { delete ui; }

void HabiTrack::populateGuiDefaults()
{
    spdlog::debug("GUI: Populating GUI defaults");
    ui->sliderOverlayUnaries->setValue(mGuiPrefs.overlayUnaries);
    ui->overlayTrackedPosition->setChecked(mGuiPrefs.overlayTrackedPos);
    ui->overlayTrajectory->setChecked(mGuiPrefs.overlayTrajectory);
    ui->trajectorySpin->setValue(mGuiPrefs.overlayTrajectoryWindow);
}

//////////////////////////////////////////////////////////////////////
// SLOTS
//////////////////////////////////////////////////////////////////////
void HabiTrack::on_sliderOverlayUnaries_sliderReleased()
{
    auto value = ui->sliderOverlayUnaries->value();
    spdlog::debug("GUI: Slider overlay unaries changed: {}", value);
    mGuiPrefs.overlayUnaries = value;
    showFrame(mCurrentFrameNumber);
}

void HabiTrack::on_overlayTrackedPosition_toggled(bool value)
{
    spdlog::debug("GUI: Overlay tracked pos changed: {}", value);
    showFrame(mCurrentFrameNumber);
}

void HabiTrack::on_overlayTrajectory_toggled(bool value)
{
    spdlog::debug("GUI: Overlay trajectory changed: {}", value);
    showFrame(mCurrentFrameNumber);
}

void HabiTrack::on_trajectorySpin_valueChanged(int value)
{
    spdlog::debug("GUI: Spin Overlay trajectory changed: {}", value);
    showFrame(mCurrentFrameNumber);
}

void HabiTrack::on_actionExpertMode_toggled(bool value)
{
    spdlog::debug("GUI: Toggled expert mode");
    ui->frameUnary->setVisible(value);
    ui->frameTracking->setVisible(value);
    ui->framePano->setVisible(value);
    ui->labelExpertMode->setVisible(value);
}

void HabiTrack::on_actionSave_Results_triggered()
{
    spdlog::debug("GUI: Save Results triggered");
    saveResults(mOutputPath / "results.yml", mPrefs);

    if (mManualUnaries.size())
        mManualUnaries.save(mUnFolder);
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

void HabiTrack::on_sliderFrame_valueChanged(int value)
{
    spdlog::debug("GUI: Changed slider frame");
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
    if (frameNumber == 0)
        return;

    auto idx = frameNumber - 1;

    spdlog::debug("GUI: Show frame {}", idx);
    mCurrentFrameNumber = frameNumber;
    cv::Mat frame = mImages.at(idx); // zero indexed here

    // set filename
    ui->labelFileName->setText(QString::fromStdString(mImages.getFileName(idx).string()));

    if (mUnaries.size())
    {
        auto scene = ui->unaryView->getUnaryScene();
        auto color = scene->getUnaryColor(idx);
        auto quality = unaryQualityToString(scene->getUnaryQuality(idx));
        ui->qualityLabel->setStyleSheet(
            QString("color: white; background-color: " + color.name() + ";"));
        ui->qualityLabel->setText(quality.c_str());
    }
    else
    {
        ui->qualityLabel->setStyleSheet(QString(""));
        ui->qualityLabel->setText("<quality>");
    }

    // overlay unary
    int unarySlider = ui->sliderOverlayUnaries->value();
    if (unarySlider > 0 && mUnaries.exists(idx))
    {
        double alpha = static_cast<double>(unarySlider) / 100.0;

        cv::Mat unary;
        if (mManualUnaries.exists(idx))
            unary = mManualUnaries.previewUnaryAt(idx);
        else
            unary = mUnaries.previewAt(idx);

        cv::Mat unaryColor;
        cv::cvtColor(unary, unaryColor, cv::COLOR_GRAY2BGR);
        std::vector<cv::Mat> channels(3);
        cv::split(unaryColor, channels);
        channels[0] = cv::Mat::zeros(unary.size(), CV_8UC1);
        cv::merge(channels, unaryColor);

        cv::resize(unaryColor, unaryColor, frame.size());

        cv::addWeighted(unaryColor, alpha, frame, (1 - alpha), 0.0, frame);
    }

    // get ant position
    cv::Point position;
    if (ui->overlayTrackedPosition->isChecked() > 0 && mDetections.exists(idx))
    {
        auto antPosition = mDetections.at(idx).position;
        /* double theta; */
        /* mTrackingData.getAntThetaAt(mCurrentFrameNumber, theta); */
        /* double thetaQuality; */
        /* mTrackingData.getAntThetaQualityAt(mCurrentFrameNumber, thetaQuality); */
        /* cv::Point dirIndicator = utils::rotatePointAroundPoint(antPosition, theta); */

        int circleThickness = 2;
        /* if (trackedPositionSlider == 100) */
        /* { */
        /*     circleThickness = 2; */
        /*     cv::line(mCurrentFrame, antPosition, dirIndicator, */
        /*         cv::Scalar(0, 255, 0), 1); */
        /* } */
        cv::Scalar color(100, 100, 255);
        cv::circle(frame, antPosition, 20, color, circleThickness);
    }

    // get trajectory
    std::size_t win = ui->trajectorySpin->value();
    if (ui->overlayTrajectory->isChecked() && win > 0 && mDetections.exists(idx))
    {
        std::size_t start = 0;
        if (idx > win)
            start = idx - win;

        auto track = mDetections.projectTo(start, idx,
            ht::matches::getTrafos(mMatchFolder, GeometricType::Homography),
            GeometricType::Homography);
        for (std::size_t i = 1; i < track.size(); ++i)
        {
            cv::Point pre = track.at(i - 1);
            cv::Point cur = track.at(i);
            cv::line(frame, pre, cur, cv::Scalar(0, 127, 255), 2);
        }

        track = mDetections.projectFrom(idx, idx + win,
            ht::matches::getTrafos(mMatchFolder, GeometricType::Homography),
            GeometricType::Homography);
        for (std::size_t i = 1; i < track.size(); ++i)
        {
            cv::Point pre = track.at(i - 1);
            cv::Point cur = track.at(i);
            cv::line(frame, pre, cur, cv::Scalar(0, 255, 255), 2);
        }
    }

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

bool HabiTrack::featureComputed() const
{
    return Features::isComputed(
        mImages, mFtFolder, mPrefs.featureType, mStartFrameNumber - 1, mEndFrameNumber);
}

bool HabiTrack::matchesComputed() const
{
    return matches::isComputed(mMatchFolder, GeometricType::Homography);
}

bool HabiTrack::unariesComputed() const
{
    return Unaries::isComputed(mImgFolder, mUnFolder, mStartFrameNumber - 1, mEndFrameNumber);
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
    QString imgFilePath = QFileDialog::getOpenFileName(
        this, tr("Open Image File List"), mStartPath, "Text (*.txt)");
    if (imgFilePath.isEmpty())
        return;
    mStartPath = imgFilePath;

    mImgFolder = fs::path(imgFilePath.toStdString());
    mImages = Images(mImgFolder);
    openImagesHelper();
}

void HabiTrack::on_actionOpenResultsFile_triggered() { }

void HabiTrack::on_buttonStartFrame_clicked()
{
    if (mCurrentFrameNumber >= mEndFrameNumber)
    {
        QMessageBox::warning(
            this, "Warning", "Start frame cannot be equal or greater than end frame");
        return;
    }
    spdlog::debug("GUI: changed start frame to {}", mCurrentFrameNumber - 1);
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
    spdlog::debug("GUI: changed end frame to {}", mCurrentFrameNumber - 1);
    // TODO: some data structures are no longer valid and should be computed again
    mEndFrameNumber = mCurrentFrameNumber;
    ui->labelEndFrame->setText(QString::number(mEndFrameNumber));
}

void HabiTrack::on_buttonExtractFeatures_clicked()
{
    spdlog::debug("GUI: Clicked Extract Features");

    auto start = mStartFrameNumber - 1;
    auto end = mEndFrameNumber;
    if (featureComputed())
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
    spdlog::debug("GUI: Clicked Extract Trafos");

    if (!featureComputed())
    {
        QMessageBox::warning(this, "Warning", "Features need to be computed first");
        return;
    }

    if (matchesComputed())
        mBar->done();
    else
    {
        matches::compute(mMatchFolder, GeometricType::Homography, mFeatures,
            matches::MatchType::Windowed, 2, 0.0, nullptr, mPrefs.cacheSize, size_t_vec(), mBar);
    }

    auto size = matches::getTrafos(mMatchFolder, GeometricType::Homography).size();
    ui->labelNumTrafos->setText(QString::number(size));

    auto types
        = ht::matches::getConnectedTypes(mMatchFolder, ht::GeometricType::Homography,
            getContinuousIds(mStartFrameNumber - 1, mEndFrameNumber));
    if (static_cast<unsigned int>(types & ht::GeometricType::Homography))
        spdlog::info("Transformations usable for unary extraction.");
    else
    {
        spdlog::warn(
            "Exracted Transformations not a continous chain, Consider increasing feature points.");
        QMessageBox::warning(this, "Warning",
            "Exracted Transformations not a continous chain, Consider increasing feature points.");
    }
}

void HabiTrack::on_buttonExtractUnaries_clicked()
{
    spdlog::debug("GUI: Clicked Extract Unaries");
    if (!matchesComputed())
    {
        QMessageBox::warning(this, "Warning", "Transformations need to be computed first");
        return;
    }

    auto start = mStartFrameNumber - 1;
    auto end = mEndFrameNumber;
    if (unariesComputed())
    {
        mUnaries = Unaries::fromDir(mImgFolder, mUnFolder, start, end);
        mBar->done();
    }
    else
    {
        auto trafos = mPrefs.removeCamMotion
            ? matches::getTrafos(mMatchFolder, GeometricType::Homography)
            : matches::PairwiseTrafos();

        mUnaries = Unaries::compute(mImgFolder, mUnFolder, start, end, mPrefs.removeRedLasers,
            mPrefs.unarySubsample, mPrefs.unarySigma, trafos, mPrefs.cacheSize, mBar);
    }
    std::vector<double> unaryQuality(mImages.size(), -1);
    for (auto i = mStartFrameNumber - 1; i < mEndFrameNumber - 1; i++)
        unaryQuality[i] = Unaries::getUnaryQuality(mUnaries.at(i));
    setUnaryScene(unaryQuality);

    ui->labelNumUnaries->setText(QString::number(mUnaries.size()));
    mManualUnaries = ManualUnaries::fromDir(mUnFolder, mPrefs.unarySubsample, mImages.getImgSize());
}

void HabiTrack::on_buttonOptimizeUnaries_clicked()
{
    mMutex.lock();
    spdlog::debug("GUI: Clicked Optimize Unaries");
    if (!unariesComputed())
    {
        QMessageBox::warning(this, "Warning", "Unaries need to be computed first");
        return;
    }

    // check queue
    // if mDetections empty, then remove all elements form queue and to -1
    int chunk;
    if (!mDetections.size())
    {
        mDetectionsQueue.clear();
        chunk = -1;
    }
    else if (mDetectionsQueue.empty())
    {
        spdlog::debug("GUI: Optimize Unaries Queue is empty");
        return;
    }
    else
    {
        chunk = mDetectionsQueue.front();
        mDetectionsQueue.pop_front();
    }

    // cancel if running already
    if (mDetectionsWatchers.count(chunk))
    {
        mDetectionsWatchers.at(chunk)->cancel();
        mDetectionsWatchers.erase(chunk);
    }

    Tracker::Settings settings;
    settings.subsample = mPrefs.unarySubsample;
    settings.pairwiseSize = mPrefs.pairwiseSize;
    settings.pairwiseSigma = mPrefs.pairwiseSigma;
    settings.manualMultiplier = mPrefs.unaryMultiplier;
    settings.calculateBearing = mPrefs.smoothBearing;
    settings.windowSize = mPrefs.smoothBearingWindowSize;
    settings.outlierTolerance = mPrefs.smoothBearingOutlierTol;

    QFuture<Detections> detectionFuture;
    if (chunk == -1)
    {
        detectionFuture = QtConcurrent::run(
            Tracker::track, mUnaries, mManualUnaries, settings, mPrefs.chunkSize);
    }
    else
    {
        detectionFuture = QtConcurrent::run(
            Tracker::track, mUnaries, mManualUnaries, settings, chunk, mPrefs.chunkSize);
    }

    auto watcher = std::make_unique<QFutureWatcher<Detections>>();
    watcher->setFuture(detectionFuture);


    connect(watcher.get(), &QFutureWatcher<Detections>::finished, this,
        [this, chunk]() { this->onDetectionsAvailable(chunk); });
    mDetectionsWatchers[chunk] = std::move(watcher);

    mMutex.unlock();

    if (!mDetectionsQueue.empty())
        on_buttonOptimizeUnaries_clicked();
}

void HabiTrack::onDetectionsAvailable(int chunkId)
{
    // TODO: is the mutex needed?
    mMutex.lock();
    auto newDetections = mDetectionsWatchers.at(chunkId)->result();
    spdlog::debug(
        "GUI: detections avaiable for chunk {} with size {}", chunkId, newDetections.size());
    mDetectionsWatchers.erase(chunkId);

    auto& dd = mDetections.data();
    for (auto&& d : newDetections.data())
        dd.insert_or_assign(d.first, std::move(d.second));
    mMutex.unlock();
}

void HabiTrack::onPositionChanged(QPointF position)
{
    if (!mUnaries.size())
        return;

    std::size_t chunk = 0;
    if (mPrefs.chunkSize)
    {
        chunk = (mCurrentFrameNumber - 1) / mPrefs.chunkSize;
        spdlog::debug("GUI: manual position changed to ({}, {}) on frame {} [chunk {}]",
            position.x(), position.y(), mCurrentFrameNumber - 1, chunk);

    }
    else
    {
        spdlog::debug("GUI: manual position changed to ({}, {}) on frame {}", position.x(),
                position.x(), position.y(), mCurrentFrameNumber - 1, chunk);
    }

    // put it in queue if it does not exist
    if (std::find(std::begin(mDetectionsQueue), std::end(mDetectionsQueue), chunk)
        == std::end(mDetectionsQueue))
    {
        spdlog::debug("GUI: added chunk {} to queue", chunk);
        mDetectionsQueue.push_back(chunk);
    }

    mManualUnaries.insert(mCurrentFrameNumber - 1, QtOpencvCore::qpoint2point(position));
    statusBar()->showMessage("Manually added unary", 1000);
    on_buttonNextFrame_clicked();
}

void HabiTrack::onBearingChanged(QPointF position)
{
    /* spdlog::debug("GUI: manual bearing changed to ({}, {}) on frame {}", position.x(), */
    /*     position.y(), mCurrentFrameNumber - 1); */
    // TODO: implement
}

void HabiTrack::onPositionCleared()
{
    if (!mUnaries.size())
        return;
    std::size_t chunk = 0;
    if (mPrefs.chunkSize)
    {
        chunk = (mCurrentFrameNumber - 1) / mPrefs.chunkSize;
        spdlog::debug(
            "GUI: manual position cleard on frame {} [chunk {}]", mCurrentFrameNumber - 1, chunk);
    }
    else
        spdlog::debug("GUI: manual position cleard on frame {}", mCurrentFrameNumber - 1);

    // put it in queue if it does not exist
    if (std::find(std::begin(mDetectionsQueue), std::end(mDetectionsQueue), chunk)
        == std::end(mDetectionsQueue))
    {
        spdlog::debug("GUI: added chunk {} to queue", chunk);
        mDetectionsQueue.push_back(chunk);
    }

    mManualUnaries.clear(mCurrentFrameNumber - 1);
    showFrame(mCurrentFrameNumber);
    statusBar()->showMessage("Unary cleared", 1000);
}

void HabiTrack::onBearingCleared()
{
    /* if (!mImages.size()) */
    /*     return; */
    /* spdlog::debug("GUI: manual bearing cleared on frame {}", mCurrentFrameNumber - 1); */
    // TODO: implement
}

void HabiTrack::setUnaryScene(std::vector<double> qualities)
{
    mUnaryQualities = qualities;

    auto num = mImages.size();
    auto scene = ui->unaryView->getUnaryScene();
    scene->setTotalImages(num);

    auto offset = mStartFrameNumber - 1;
    num = mEndFrameNumber - mStartFrameNumber;
    std::size_t chunkSize = 100;
    for (std::size_t i = 0; i < std::ceil(static_cast<double>(num) / chunkSize); i++)
    {
        auto start = i * chunkSize + offset;
        auto end = std::min((i + 1) * chunkSize, num) + offset;
        auto mid = start + (end - start) / 2;
        std::nth_element(std::begin(qualities) + start, std::begin(qualities) + mid,
            std::begin(qualities) + end);
        auto median = qualities[mid];

        std::vector<double> dists;
        dists.reserve(end - start);
        for (std::size_t j = start; j < end; j++)
            dists.push_back(std::abs(mUnaryQualities[j] - median));

        std::nth_element(std::begin(dists), std::begin(dists) + dists.size() / 2, std::end(dists));
        auto medianDist = dists[dists.size() / 2];
        spdlog::debug("GUI: Unary low threshold {}", median + 2.0 * medianDist);
        spdlog::debug("GUI: Unary high threshold {}", median + 4.0 * medianDist);
        for (std::size_t j = start; j < end; j++)
        {
            if (mUnaryQualities[j] < (median + 1.5 * medianDist))
                scene->setUnaryQuality(j, UnaryQuality::Good);
            else if (mUnaryQualities[j] < (median + 3.0 * medianDist))
                scene->setUnaryQuality(j, UnaryQuality::Poor);
            else
                scene->setUnaryQuality(j, UnaryQuality::Critical);
        }
    }
}

} // namespace gui
