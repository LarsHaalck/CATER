#include "gui/mainWindow.h"
#include "ui_mainWindow.h"

#include "gui/preferencesDialog.h"
#include "gui/progressStatusBar.h"
#include "gui/qtOpencvCore.h"
#include "habitrack/tracker.h"
#include "image-processing/util.h"
#include "resultsIO.h"
#include "setIO.h"
#include <QDate>
#include <QFileDialog>
#include <QMessageBox>
#include <QSound>
#include <QStatusBar>
#include <QTime>
#include <QtConcurrent>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

using namespace ht;
namespace fs = std::filesystem;

constexpr int statusDelay = 2000; // used

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
    , mInvisibles()
    , mDetections()
    , mDetectionsWatchers()
    , mDetectionsQueue()
{
    ui->setupUi(this);
    statusBar()->showMessage("", statusDelay);
    populateGuiDefaults();

    ui->unaryView->setOptimizationFlags(QGraphicsView::DontSavePainterState
        | QGraphicsView::DontAdjustForAntialiasing | QGraphicsView::IndirectPainting);

    // needs to be done after setupUi
    mScene = ui->graphicsView->getTrackerScene();
    ui->graphicsView->setDragMode(QGraphicsView::ScrollHandDrag);
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
    ui->overlayBearings->setChecked(mGuiPrefs.overlayBearing);
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

void HabiTrack::on_overlayBearings_toggled(bool value)
{
    spdlog::debug("GUI: Overlay bearing changed: {}", value);
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

    saveResults(mResultsFile, mPrefs, mImgFolder, mStartFrameNumber, mEndFrameNumber);
    if (mManualUnaries.size())
        mManualUnaries.save(mUnFolder);
    if (mInvisibles.size())
        saveSet(mSetFile, mInvisibles);

    //mManualBearing.save(mOutputPath);

    statusBar()->showMessage("Saved results.", statusDelay);
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

        // TODO: should this be saved implicitly?
        saveResults(mResultsFile, mPrefs, mImgFolder, mStartFrameNumber, mEndFrameNumber);
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

void HabiTrack::populatePaths()
{
    spdlog::debug("Generated output path: {}", mOutputPath.string());

    mResultsFile = mOutputPath / "results.yml";
    mFtFolder = mOutputPath / "fts";
    mMatchFolder = mOutputPath / "matches";
    mUnFolder = mOutputPath / "unaries";
    mDetectionsFile = mOutputPath / "detections.yml";
    mSetFile = mOutputPath / "invisibles.yml";

    try
    {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(spdlog::level::info);

        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
            (mOutputPath / "logs.txt").string(), false);
        file_sink->set_level(spdlog::level::debug);

        auto logger = std::make_shared<spdlog::logger>(
            "multi_sink", spdlog::sinks_init_list({console_sink, file_sink}));
        logger->set_level(spdlog::level::debug);
        spdlog::flush_every(std::chrono::seconds(3));
        spdlog::set_default_logger(logger);
        spdlog::info("New run -------------");
    }
    catch (const spdlog::spdlog_ex& ex)
    {
        std::cout << "Log initialization failed: " << ex.what() << std::endl;
    }
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
    ui->labelLabel->setText(mInvisibles.count(idx) ? QString("Invisible") : QString());

    if (mUnaries.size())
    {
        auto scene = ui->unaryView->getUnaryScene();
        auto color = scene->getUnaryColor(idx);
        auto quality = UnaryScene::unaryQualityToString(scene->getUnaryQuality(idx));
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

    // get position
    if (ui->overlayTrackedPosition->isChecked() > 0 && mDetections.exists(idx))
    {
        auto detection = mDetections.at(idx);
        auto position = detection.position;
        auto theta = detection.theta;
        auto dirIndicator = rotatePointAroundPoint(position, theta);

        if (ui->overlayBearings->isChecked())
            cv::line(frame, position, dirIndicator, cv::Scalar(0, 255, 0), 1);

        // overloay manual unary as well
        if (mManualUnaries.exists(idx))
        {
            auto unary = mManualUnaries.previewUnaryAt(idx);
            cv::Mat unaryColor;
            cv::cvtColor(unary, unaryColor, cv::COLOR_GRAY2BGR);
            std::vector<cv::Mat> channels(3);
            cv::split(unaryColor, channels);
            channels[2] = cv::Mat::zeros(unary.size(), CV_8UC1);
            cv::merge(channels, unaryColor);
            cv::resize(unaryColor, unaryColor, frame.size());
            cv::add(frame, unaryColor, frame);
        }

        cv::Scalar color;
        if (mInvisibles.count(idx))
            color = cv::Scalar(100, 255, 255);
        else
            color = cv::Scalar(100, 100, 255);

        cv::circle(frame, position, 20, color, 2);
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

void HabiTrack::openImagesHelper(const fs::path& path)
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

    ui->labelStartFrame->setText(QString::number(mStartFrameNumber));
    ui->labelEndFrame->setText(QString::number(mEndFrameNumber));
    ui->labelNumTrafos->setText(QString::number(0));
    ui->labelNumUnaries->setText(QString::number(0));
    ui->labelNumTrackedPos->setText(QString::number(0));

    ui->buttonPrevFrame->setEnabled(true);
    ui->buttonNextFrame->setEnabled(true);
    ui->buttonVisible->setEnabled(true);
    ui->buttonInvisible->setEnabled(true);
    ui->actionPrev_Frame->setEnabled(true);
    ui->actionNext_Frame->setEnabled(true);

    ui->buttonTrack->setEnabled(true);
    ui->frameTracking->setEnabled(true);
    for (auto& child : ui->frameTracking->findChildren<QWidget*>())
        child->setEnabled(true);

    ui->buttonStartFrame->setEnabled(true);
    ui->buttonEndFrame->setEnabled(true);

    // show first frame
    mCurrentFrameNumber = mStartFrameNumber;
    showFrame(mStartFrameNumber);

    if (path.empty())
    {
        auto tempPath = fs::path(mStartPath.toStdString());
        if (fs::is_directory(tempPath))
            mOutputPath = tempPath;
        else
            mOutputPath = tempPath.filename();

        mOutputPath += "_output";
        auto date = QDate::currentDate().toString("yyyy-MM-dd").toStdString();
        auto time = QTime::currentTime().toString("hh-mm-ss").toStdString();

        mOutputPath /= "now";
        /* mOutputPath /= date + "_" + time; */
    }
    else
        mOutputPath = path;

    populatePaths();
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

bool HabiTrack::detectsComputed() const { return fs::is_regular_file(mDetectionsFile); }

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
    mStartFrameNumber = 1;
    mEndFrameNumber = mImages.size();
    openImagesHelper();
}

void HabiTrack::on_actionOpenImgList_triggered()
{
    QMessageBox::warning(this, "Warning", "Not implemented yet");
    return;

    /* spdlog::debug("GUI: Triggered open image list file"); */
    /* QString imgFilePath = QFileDialog::getOpenFileName( */
    /*     this, tr("Open Image File List"), mStartPath, "Text (*.txt)"); */
    /* if (imgFilePath.isEmpty()) */
    /*     return; */
    /* auto parentPath = fs::path(imgFilePath.toStdString()).parent_path(); */
    /* mStartPath = QString::fromStdString(parentPath.string()); */

    /* mImgFolder = fs::path(imgFilePath.toStdString()); */
    /* mImages = Images(mImgFolder); */
    /* openImagesHelper(); */
}

void HabiTrack::on_actionOpenResultsFile_triggered()
{
    spdlog::debug("GUI: Triggered Open results file");
    QString resultFile = QFileDialog::getOpenFileName(
        this, tr("Open Image File List"), mStartPath, "YAML (*.yml)");
    if (resultFile.isEmpty())
        return;

    auto parentPath = fs::path(resultFile.toStdString()).parent_path();
    mStartPath = QString::fromStdString(parentPath.string());

    spdlog::debug("new start path: {}", mStartPath.toStdString());
    auto [prefs_, imgFolder_, start_, end_] = loadResults(resultFile.toStdString());
    mPrefs = prefs_;
    mImgFolder = imgFolder_;
    mStartFrameNumber = start_;
    mEndFrameNumber = end_;

    mImages = Images(mImgFolder);
    openImagesHelper(parentPath);

    if (featureComputed())
    {
        on_buttonExtractFeatures_clicked();
        if (matchesComputed())
        {
            on_buttonExtractTrafos_clicked();
            if (unariesComputed())
            {
                on_buttonExtractUnaries_clicked();
                if (detectsComputed())
                {
                    mDetections = Detections::fromDir(mDetectionsFile);
                }
            }
        }
    }
    if (fs::is_regular_file(mSetFile))
        mInvisibles = loadSet(mSetFile);
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

void HabiTrack::on_buttonVisible_clicked()
{
    spdlog::debug("GUI: Removed invisible tag for: {}", mCurrentFrameNumber - 1);
    auto it = mInvisibles.find(mCurrentFrameNumber - 1);
    if (it != std::end(mInvisibles))
        mInvisibles.erase(it);
    statusBar()->showMessage("Set detection to visible", statusDelay);
}

void HabiTrack::on_buttonInvisible_clicked()
{
    spdlog::debug("GUI: Inserted invisible tag for: {}", mCurrentFrameNumber - 1);
    mInvisibles.insert(mCurrentFrameNumber - 1);
    showFrame(mCurrentFrameNumber);
    statusBar()->showMessage("Set detection to invisible", statusDelay);
}

void HabiTrack::on_mikeButton_clicked()
{
    on_buttonExtractFeatures_clicked();
    on_buttonExtractTrafos_clicked();
    on_buttonExtractUnaries_clicked();
    on_buttonOptimizeUnaries_clicked();
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
            matches::MatchType::Windowed, 2, 0.0, nullptr, mPrefs.cacheSize,
            getContinuousIds(mStartFrameNumber - 1, mEndFrameNumber), mBar);
    }

    auto size = matches::getTrafos(mMatchFolder, GeometricType::Homography).size();
    ui->labelNumTrafos->setText(QString::number(size));

    auto types = ht::matches::getConnectedTypes(mMatchFolder, ht::GeometricType::Homography,
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

    // zero init if existent
    mManualUnaries = ManualUnaries::fromDir(mUnFolder, mPrefs.unarySubsample, mImages.getImgSize());

    setupUnaryScene(unaryQuality);
    ui->labelNumUnaries->setText(QString::number(mUnaries.size()));
}

void HabiTrack::on_buttonOptimizeUnaries_clicked()
{
    {
        QMutexLocker locker(&mMutex);

        // save manual unaries if existent
        if (mManualUnaries.size())
            mManualUnaries.save(mUnFolder);

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
            QMessageBox::information(
                this, "Information", "No new manual unaries supplied. Nothing to do.");
            return;
        }
        else
        {
            chunk = mDetectionsQueue.front();
            mDetectionsQueue.pop_front();
        }

        toggleChunkUnaryScene(chunk, true);

        // TODO: doesn't work because QtConcurrent::run cannot be cancelled
        // cancel if running already
        /* if (mDetectionsWatchers.count(chunk)) */
        /* { */
        /*     mDetectionsWatchers.at(chunk)->cancel(); */
        /*     mDetectionsWatchers.at(chunk)->waitForFinished(); */
        /*     spdlog::debug("Running {}", mDetectionsWatchers.at(chunk)->isRunning()); */
        /*     mDetectionsWatchers.erase(chunk); */
        /* } */

        Tracker::Settings settings;
        settings.subsample = mPrefs.unarySubsample;
        settings.pairwiseSize = mPrefs.pairwiseSize;
        settings.pairwiseSigma = mPrefs.pairwiseSigma;
        settings.manualMultiplier = mPrefs.unaryMultiplier;
        settings.smoothBearing = mPrefs.smoothBearing;
        settings.windowSize = mPrefs.smoothBearingWindowSize;
        settings.outlierTolerance = mPrefs.smoothBearingOutlierTol;
        settings.chunkSize = mPrefs.chunkSize;

        QFuture<Detections> detectionFuture;
        auto trafos = ht::matches::getTrafos(mMatchFolder, GeometricType::Homography);
        if (chunk == -1)
        {
            detectionFuture
                = QtConcurrent::run(Tracker::track, mUnaries, mManualUnaries, settings, trafos);
        }
        else
        {
            detectionFuture = QtConcurrent::run(
                Tracker::track, mUnaries, mManualUnaries, settings, chunk, trafos);
        }

        auto watcher = std::make_unique<QFutureWatcher<Detections>>();
        watcher->setFuture(detectionFuture);

        connect(watcher.get(), &QFutureWatcher<Detections>::finished, this,
            [this, chunk]() { this->onDetectionsAvailable(chunk); });
        mDetectionsWatchers[chunk] = std::move(watcher);
    }

    if (!mDetectionsQueue.empty())
        on_buttonOptimizeUnaries_clicked();
}

void HabiTrack::onDetectionsAvailable(int chunkId)
{
    // TODO: is the mutex needed?
    QMutexLocker locker(&mMutex);
    QSound::play("qrc:///sounds/notification.wav");
    statusBar()->showMessage("New detections available", statusDelay);

    toggleChunkUnaryScene(chunkId, false);
    auto newDetections = mDetectionsWatchers.at(chunkId)->result();
    spdlog::debug(
        "GUI: detections avaiable for chunk {} with size {}", chunkId, newDetections.size());
    mDetectionsWatchers.erase(chunkId);

    auto& dd = mDetections.data();
    for (auto&& d : newDetections.data())
        dd.insert_or_assign(d.first, std::move(d.second));

    // save detections
    mDetections.save(mDetectionsFile);
}

void HabiTrack::onPositionChanged(QPointF position)
{
    if (!mUnaries.size())
        return;

    std::size_t chunk = 0;
    if (mPrefs.chunkSize)
    {
        chunk = (mCurrentFrameNumber - mStartFrameNumber) / mPrefs.chunkSize;
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

    ui->unaryView->getUnaryScene()->setUnaryQuality(
        mCurrentFrameNumber - 1, UnaryQuality::Excellent);
    /* ui->unaryView->getUnaryScene()->update(); */
    statusBar()->showMessage("Manually added unary", statusDelay);
    on_buttonNextFrame_clicked();
}

void HabiTrack::onBearingChanged(QPointF)
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
        chunk = (mCurrentFrameNumber - mStartFrameNumber) / mPrefs.chunkSize;
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
    statusBar()->showMessage("Manual unary cleared", statusDelay);

    ui->unaryView->getUnaryScene()->setUnaryQuality(
        mCurrentFrameNumber - 1, mUnaryQualityValues[mCurrentFrameNumber - 1]);
    ui->unaryView->getUnaryScene()->update();
}

void HabiTrack::onBearingCleared()
{
    /* if (!mImages.size()) */
    /*     return; */
    /* spdlog::debug("GUI: manual bearing cleared on frame {}", mCurrentFrameNumber - 1); */
    // TODO: implement
}

// this is by value by choice, because it needs to be copied and modifed in-place
void HabiTrack::setupUnaryScene(std::vector<double> qualities)
{
    mUnaryQualities = qualities;

    auto num = mImages.size();
    auto scene = ui->unaryView->getUnaryScene();
    scene->setTotalImages(num);

    auto offset = mStartFrameNumber - 1;
    num = mEndFrameNumber - mStartFrameNumber;
    std::size_t medSize = 100;
    mBar->setTotal(num);
    for (std::size_t i = 0; i < std::ceil(static_cast<double>(num) / medSize); i++)
    {
        auto start = i * medSize + offset;
        auto end = std::min((i + 1) * medSize, num) + offset;
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
            UnaryQuality qual;
            if (mUnaryQualities[j] < (median + 1.5 * medianDist))
                qual = UnaryQuality::Good;
            else if (mUnaryQualities[j] < (median + 3.0 * medianDist))
                qual = UnaryQuality::Poor;
            else
                qual = UnaryQuality::Critical;

            scene->setUnaryQuality(j, qual);
            mUnaryQualityValues[j] = qual;
            mBar->inc();
        }
    }
    mBar->done();

    // this only works for continous unaries (which should always be the case when using the gui)
    // set all cunks on not-computing
    toggleChunkUnaryScene(-1, false);

    for (const auto& manualUnary : std::as_const(mManualUnaries))
        scene->setUnaryQuality(manualUnary.first, UnaryQuality::Excellent);
}

void HabiTrack::toggleChunkUnaryScene(int chunkId, bool computing)
{
    auto numUnaries = mUnaries.size();
    auto chunkSize = mPrefs.chunkSize;
    auto numChunks = Tracker::getNumChunks(numUnaries, chunkSize);

    std::vector<int> chunks;
    if (chunkId == -1)
    {
        chunks.resize(numChunks);
        std::iota(std::begin(chunks), std::end(chunks), 0);
    }
    else
        chunks = {chunkId};

    for (auto chunk : chunks)
    {
        UnaryState state;
        if (computing)
            state = (chunk % 2) ? UnaryState::ComputingAlt : UnaryState::Computing;
        else
            state = (chunk % 2) ? UnaryState::DefaultAlt : UnaryState::Default;

        auto end = Tracker::getChunkEnd(chunk, numChunks, chunkSize, numUnaries);
        for (std::size_t j = chunk * chunkSize; j < end; j++)
            ui->unaryView->getUnaryScene()->setUnaryState(j + mStartFrameNumber - 1, state);
    }

    ui->unaryView->getUnaryScene()->update();
}

} // namespace gui
