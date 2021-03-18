#include "gui/mainWindow.h"
#include "ui_mainWindow.h"

#include "gui/imagesWorker.h"
#include "gui/preferencesDialog.h"
#include "gui/qtOpencvCore.h"
#include "gui/scopedBlocker.h"
#include "image-processing/util.h"
#include <QDate>
#include <QFileDialog>
#include <QMessageBox>
#ifdef WITH_QT5_MULTIMEDIA
#include <QSound>
#endif
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

constexpr int statusDelay = 2000;

namespace gui
{
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , mHabiTrack()
    , mViewer(mHabiTrack.images(), mHabiTrack.unaries(), mHabiTrack.manualUnaries(),
          mHabiTrack.detections())
{
    ui->setupUi(this);

    statusBar()->showMessage("", statusDelay);
    populateGuiDefaults();

    ui->unaryView->setOptimizationFlags(QGraphicsView::DontSavePainterState
        | QGraphicsView::DontAdjustForAntialiasing | QGraphicsView::IndirectPainting);

    // needs to be done after setupUi
    mScene = ui->graphicsView->getTrackerScene();
    ui->graphicsView->setDragMode(QGraphicsView::ScrollHandDrag);
    ui->graphicsView->setFocusPolicy(Qt::NoFocus);

    setupProgressBar();

    /* connect(ui->unaryView, &UnaryGraphicsView::jumpedToUnary, this, */
    /*     [=](std::size_t num) { this->showFrame(num + 1); }); */

    connect(this->ui->graphicsView, SIGNAL(positionChanged(QPointF)), this,
        SLOT(onPositionChanged(QPointF)));
    connect(this->ui->graphicsView, SIGNAL(bearingChanged(QPointF)), this,
        SLOT(onBearingChanged(QPointF)));
    connect(this->ui->graphicsView, SIGNAL(positionCleared()), this, SLOT(onPositionCleared()));
    connect(this->ui->graphicsView, SIGNAL(bearingCleared()), this, SLOT(onBearingCleared()));
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::setupProgressBar()
{
    mBar = std::make_shared<ProgressStatusBar>(this);
    connect(mBar.get(), SIGNAL(totalChanged(int)), this, SLOT(onTotalChanged(int)));
    connect(mBar.get(), SIGNAL(incremented(int)), this, SLOT(onIncremented(int)));
    connect(mBar.get(), SIGNAL(incremented()), this, SLOT(onIncremented()));
    connect(mBar.get(), SIGNAL(isDone()), this, SLOT(onIsDone()));
    connect(mBar.get(), SIGNAL(statusChanged(const QString&)), this,
        SLOT(onStatusChanged(const QString&)));
    mHabiTrack.setProgressBar(mBar);
}

void MainWindow::populateGuiDefaults()
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
void MainWindow::on_sliderOverlayUnaries_sliderReleased()
{
    auto value = ui->sliderOverlayUnaries->value();
    spdlog::debug("GUI: Slider overlay unaries changed: {}", value);
    mGuiPrefs.overlayUnaries = value;

    // TODO
    /* showFrame(mCurrentFrameNumber); */
}

void MainWindow::on_overlayTrackedPosition_toggled(bool value)
{
    spdlog::debug("GUI: Overlay tracked pos changed: {}", value);

    // TODO
    /* showFrame(mCurrentFrameNumber); */
}

void MainWindow::on_overlayBearings_toggled(bool value)
{
    spdlog::debug("GUI: Overlay bearing changed: {}", value);

    // TODO
    /* showFrame(mCurrentFrameNumber); */
}

void MainWindow::on_overlayTrajectory_toggled(bool value)
{
    spdlog::debug("GUI: Overlay trajectory changed: {}", value);

    // TODO
    /* showFrame(mCurrentFrameNumber); */
}

void MainWindow::on_trajectorySpin_valueChanged(int value)
{
    spdlog::debug("GUI: Spin Overlay trajectory changed: {}", value);

    // TODO
    /* showFrame(mCurrentFrameNumber); */
}

void MainWindow::on_actionExpertMode_toggled(bool value)
{
    spdlog::debug("GUI: Toggled expert mode");
    ui->frameUnary->setVisible(value);
    ui->frameTracking->setVisible(value);
    ui->framePano->setVisible(value);
    ui->labelExpertMode->setVisible(value);
}

void MainWindow::on_actionSave_Results_triggered()
{
    /* spdlog::debug("GUI: Save Results triggered"); */

    /* saveResults(mResultsFile, mPrefs, mImgFolder, mStartFrameNumber, mEndFrameNumber); */
    /* if (mManualUnaries.size()) */
    /*     mManualUnaries.save(mUnFolder); */
    /* if (mInvisibles.size()) */
    /*     saveSet(mSetFile, mInvisibles); */

    /* //mManualBearing.save(mOutputPath); */

    /* statusBar()->showMessage("Saved results.", statusDelay); */
}

void MainWindow::on_actionLabel_Editor_triggered()
{
    /* LabelDialog labelDialog(this); */
    /* labelDialog.exec(); */
}

void MainWindow::on_actionPreferences_triggered()
{
    /* PreferencesDialog prefDialog(this, mPrefs); */
    /* auto code = prefDialog.exec(); */
    /* if (code == QDialog::Accepted) */
    /* { */
    /*     // overwrite current settings if accepted, discard otherwise */
    /*     mPrefs = prefDialog.getPreferences(); */
    /*     spdlog::debug("GUI: Changed Preferences to: {}", mPrefs); */

    /*     // TODO: should this be saved implicitly? */
    /*     saveResults(mResultsFile, mPrefs, mImgFolder, mStartFrameNumber, mEndFrameNumber); */
    /* } */
}

void MainWindow::on_sliderFrame_valueChanged(int value)
{
    showFrame(value);
}

void MainWindow::on_spinCurrentFrame_valueChanged(int value)
{
    showFrame(value);
}

void MainWindow::on_buttonPrevFrame_clicked()
{
    if (mCurrentFrameNumber > 1)
    {
        auto newIdx = mCurrentFrameNumber - 1;
        showFrame(newIdx);
    }
}

void MainWindow::on_buttonNextFrame_clicked()
{
    if (mCurrentFrameNumber < mHabiTrack.images().size())
    {
        auto newIdx = mCurrentFrameNumber + 1;
        showFrame(newIdx);
    }
}

void MainWindow::on_actionPrev_Frame_triggered()
{
    // mask action to not overshoot target frame
    this->ui->actionPrev_Frame->setDisabled(true);
    on_buttonPrevFrame_clicked();
    this->ui->actionPrev_Frame->setDisabled(false);
}

void MainWindow::on_actionNext_Frame_triggered()
{
    // mask action to not overshoot target frame
    this->ui->actionNext_Frame->setDisabled(true);
    on_buttonNextFrame_clicked();
    this->ui->actionNext_Frame->setDisabled(false);
}

void MainWindow::updateSlider()
{
    auto blocker = ScopedBlocker{ui->sliderFrame, ui->spinCurrentFrame};
    ui->sliderFrame->setValue(mCurrentFrameNumber);
    ui->spinCurrentFrame->setValue(mCurrentFrameNumber);
}

void MainWindow::showFrame(std::size_t frame)
{
    mCurrentFrameNumber = frame;
    auto img = mViewer.getFrame(frame - 1);
    auto pixMap = QPixmap::fromImage(QtOpencvCore::img2qimgRaw(img));
    mScene->setPixmap(pixMap);
    updateSlider();
}

void MainWindow::openImagesHelper()
{
    auto imgs = mHabiTrack.images();
    auto numImgs = imgs.size();

    auto blocker = ScopedBlocker {
        ui->sliderFrame, ui->spinCurrentFrame, ui->buttonPrevFrame, ui->buttonNextFrame};

    // set up gui elements
    ui->labelMaxFrames->setText(QString::number(numImgs));
    ui->sliderFrame->setMinimum(1);
    ui->sliderFrame->setMaximum(numImgs);
    ui->sliderFrame->setValue(1);
    ui->sliderFrame->setEnabled(true);

    ui->spinCurrentFrame->setMinimum(1);
    ui->spinCurrentFrame->setMaximum(numImgs);
    ui->spinCurrentFrame->setValue(1);
    ui->spinCurrentFrame->setKeyboardTracking(false);
    ui->spinCurrentFrame->setEnabled(true);

    ui->labelFileName->setText(QString::fromStdString(imgs.getFileName(0)));

    ui->labelStartFrame->setText(QString::number(mHabiTrack.getStartFrame()));
    ui->labelEndFrame->setText(QString::number(mHabiTrack.getEndFrame()));
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
    mCurrentFrameNumber = mHabiTrack.getStartFrame();
    showFrame(mCurrentFrameNumber);
}

void MainWindow::on_actionOpenImgFolder_triggered()
{
    spdlog::debug("GUI: Triggered Open image folder");
    QString imgFolderPath = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
        mStartPath, QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (imgFolderPath.isEmpty())
        return;

    mBar->setTotal(0);
    mHabiTrack.loadImageFolder(fs::path(imgFolderPath.toStdString()));
    openImagesHelper();
    mBar->setTotal(1);
}

void MainWindow::on_actionOpenImgList_triggered()
{
    QMessageBox::warning(this, "Warning", "Not implemented yet");
    return;
}

void MainWindow::on_actionOpenResultsFile_triggered()
{
    spdlog::debug("GUI: Triggered Open results file");
    QString resultFile = QFileDialog::getOpenFileName(
        this, tr("Open Image File List"), mStartPath, "YAML (*.yml)");
    if (resultFile.isEmpty())
        return;

    mHabiTrack.loadResultsFile(resultFile.toStdString());

    // TODO: show start frame number frame
}

void MainWindow::on_buttonStartFrame_clicked()
{
    if (mCurrentFrameNumber >= mHabiTrack.getEndFrame())
    {
        QMessageBox::warning(
            this, "Warning", "Start frame cannot be equal or greater than end frame");
        return;
    }
    spdlog::debug("GUI: changed start frame to {}", mCurrentFrameNumber);
    // TODO: some data structures are no longer valid and should be computed again
    mHabiTrack.setStartFrame(mCurrentFrameNumber);
    ui->labelStartFrame->setText(QString::number(mCurrentFrameNumber));
}

void MainWindow::on_buttonEndFrame_clicked()
{
    if (mCurrentFrameNumber <= mHabiTrack.getStartFrame())
    {
        QMessageBox::warning(
            this, "Warning", "End frame cannot be equal or smaller than start frame");
        return;
    }
    spdlog::debug("GUI: changed end frame to {}", mCurrentFrameNumber);
    // TODO: some data structures are no longer valid and should be computed again
    mHabiTrack.setEndFrame(mCurrentFrameNumber);
    ui->labelEndFrame->setText(QString::number(mCurrentFrameNumber));
}

void MainWindow::on_mikeButton_clicked()
{
    on_buttonExtractFeatures_clicked();
    on_buttonExtractTrafos_clicked();
    on_buttonExtractUnaries_clicked();
    on_buttonOptimizeUnaries_clicked();
}

void MainWindow::on_buttonExtractFeatures_clicked()
{
    spdlog::debug("GUI: Clicked Extract Features");
    mBlockingThread = QThread::create(&HabiTrack::extractFeatures, &mHabiTrack);
    mBlockingThread->start();
}

void MainWindow::on_buttonExtractTrafos_clicked()
{
    if (!mHabiTrack.featureComputed())
    {
        QMessageBox::warning(this, "Warning", "Features need to be computed first");
        return;
    }

    mBlockingThread = QThread::create(&HabiTrack::extractTrafos, &mHabiTrack);
    connect(mBlockingThread, SIGNAL(finished()), this, SLOT(on_trafosExtracted()));
    mBlockingThread->start();

}

void MainWindow::on_trafosExtracted()
{

    /* auto size = mHabiTrack. */
    /* auto size = matches::getTrafos(mMatchFolder, GeometricType::Homography).size(); */

    /* ui->labelNumTrafos->setText(QString::number(size)); */
    QMessageBox::warning(this, "Warning",
        "Exracted Transformations not a continous chain, Consider increasing feature points.");
}

void MainWindow::on_buttonExtractUnaries_clicked()
{
    /* spdlog::debug("GUI: Clicked Extract Unaries"); */
    /* if (!matchesComputed()) */
    /* { */
    /*     QMessageBox::warning(this, "Warning", "Transformations need to be computed first"); */
    /*     return; */
    /* } */

    /* auto start = mStartFrameNumber - 1; */
    /* auto end = mEndFrameNumber; */
    /* if (unariesComputed()) */
    /* { */
    /*     mUnaries = Unaries::fromDir(mImgFolder, mUnFolder, start, end); */
    /*     mBar->done(); */
    /* } */
    /* else */
    /* { */
    /*     auto trafos = mPrefs.removeCamMotion */
    /*         ? matches::getTrafos(mMatchFolder, GeometricType::Homography) */
    /*         : matches::PairwiseTrafos(); */

    /*     mUnaries = Unaries::compute(mImgFolder, mUnFolder, start, end, mPrefs.removeRedLasers, */
    /*         mPrefs.unarySubsample, mPrefs.unarySigma, trafos, mPrefs.cacheSize, mBar); */
    /* } */
    /* std::vector<double> unaryQuality(mImages.size(), -1); */
    /* for (auto i = mStartFrameNumber - 1; i < mEndFrameNumber - 1; i++) */
    /*     unaryQuality[i] = Unaries::getUnaryQuality(mUnaries.at(i)); */

    /* // zero init if existent */
    /* mManualUnaries = ManualUnaries::fromDir( */
    /*     mUnFolder, mPrefs.unarySubsample, mPrefs.manualUnarySize, mImages.getImgSize()); */

    /* setupUnaryScene(unaryQuality); */
    /* ui->labelNumUnaries->setText(QString::number(mUnaries.size())); */
}

void MainWindow::on_buttonOptimizeUnaries_clicked()
{
    /* { */
    /*     QMutexLocker locker(&mMutex); */

    /*     // save manual unaries if existent */
    /*     if (mManualUnaries.size()) */
    /*         mManualUnaries.save(mUnFolder); */

    /*     spdlog::debug("GUI: Clicked Optimize Unaries"); */
    /*     if (!unariesComputed()) */
    /*     { */
    /*         QMessageBox::warning(this, "Warning", "Unaries need to be computed first"); */
    /*         return; */
    /*     } */

    /*     // check queue */
    /*     // if mDetections empty, then remove all elements form queue and to -1 */
    /*     int chunk; */
    /*     if (!mDetections.size()) */
    /*     { */
    /*         mDetectionsQueue.clear(); */
    /*         chunk = -1; */
    /*     } */
    /*     else if (mDetectionsQueue.empty()) */
    /*     { */
    /*         spdlog::debug("GUI: Optimize Unaries Queue is empty"); */
    /*         QMessageBox::information( */
    /*             this, "Information", "No new manual unaries supplied. Nothing to do."); */
    /*         return; */
    /*     } */
    /*     else */
    /*     { */
    /*         chunk = mDetectionsQueue.front(); */
    /*         mDetectionsQueue.pop_front(); */
    /*     } */

    /*     toggleChunkUnaryScene(chunk, true); */

    /*     // TODO: doesn't work because QtConcurrent::run cannot be cancelled */
    /*     // cancel if running already */
    /*     /1* if (mDetectionsWatchers.count(chunk)) *1/ */
    /*     /1* { *1/ */
    /*     /1*     mDetectionsWatchers.at(chunk)->cancel(); *1/ */
    /*     /1*     mDetectionsWatchers.at(chunk)->waitForFinished(); *1/ */
    /*     /1*     spdlog::debug("Running {}", mDetectionsWatchers.at(chunk)->isRunning()); *1/ */
    /*     /1*     mDetectionsWatchers.erase(chunk); *1/ */
    /*     /1* } *1/ */

    /*     Tracker::Settings settings; */
    /*     settings.subsample = mPrefs.unarySubsample; */
    /*     settings.pairwiseSize = mPrefs.pairwiseSize; */
    /*     settings.pairwiseSigma = mPrefs.pairwiseSigma; */
    /*     settings.manualMultiplier = mPrefs.unaryMultiplier; */
    /*     settings.smoothBearing = mPrefs.smoothBearing; */
    /*     settings.windowSize = mPrefs.smoothBearingWindowSize; */
    /*     settings.outlierTolerance = mPrefs.smoothBearingOutlierTol; */
    /*     settings.chunkSize = mPrefs.chunkSize; */

    /*     QFuture<Detections> detectionFuture; */
    /*     auto trafos = ht::matches::getTrafos(mMatchFolder, GeometricType::Homography); */
    /*     if (chunk == -1) */
    /*     { */
    /*         detectionFuture */
    /*             = QtConcurrent::run(Tracker::track, mUnaries, mManualUnaries, settings, trafos); */
    /*     } */
    /*     else */
    /*     { */
    /*         detectionFuture = QtConcurrent::run( */
    /*             Tracker::track, mUnaries, mManualUnaries, settings, chunk, trafos); */
    /*     } */

    /*     auto watcher = std::make_unique<QFutureWatcher<Detections>>(); */
    /*     watcher->setFuture(detectionFuture); */

    /*     connect(watcher.get(), &QFutureWatcher<Detections>::finished, this, */
    /*         [this, chunk]() { this->onDetectionsAvailable(chunk); }); */
    /*     mDetectionsWatchers[chunk] = std::move(watcher); */
    /* } */

    /* if (!mDetectionsQueue.empty()) */
    /*     on_buttonOptimizeUnaries_clicked(); */
}

void MainWindow::onDetectionsAvailable(int chunkId)
{
    /* // TODO: is the mutex needed? */
    /* QMutexLocker locker(&mMutex); */

    /* #ifdef WITH_QT5_MULTIMEDIA */
    /* QSound::play("qrc:///sounds/notification.wav"); */
    /* #endif */

    /* statusBar()->showMessage("New detections available", statusDelay); */

    /* toggleChunkUnaryScene(chunkId, false); */
    /* auto newDetections = mDetectionsWatchers.at(chunkId)->result(); */
    /* spdlog::debug( */
    /*     "GUI: detections avaiable for chunk {} with size {}", chunkId, newDetections.size()); */
    /* mDetectionsWatchers.erase(chunkId); */

    /* auto& dd = mDetections.data(); */
    /* for (auto&& d : newDetections.data()) */
    /*     dd.insert_or_assign(d.first, std::move(d.second)); */

    /* // save detections */
    /* mDetections.save(mDetectionsFile); */
}

void MainWindow::onTotalChanged(int total)
{
    ui->progressBar->setValue(0);
    ui->progressBar->setMaximum(total);
}

void MainWindow::onIncremented() { ui->progressBar->setValue(ui->progressBar->value() + 1); }

void MainWindow::onIncremented(int inc)
{
    ui->progressBar->setValue(ui->progressBar->value() + inc);
}

void MainWindow::onIsDone()
{
    ui->labelProgress->setText("Finished");
    ui->progressBar->setValue(ui->progressBar->maximum());
}

void MainWindow::onStatusChanged(const QString& state)
{
    ui->labelProgress->setText(state);
}

void MainWindow::onPositionChanged(QPointF position)
{
    /* if (!mUnaries.size()) */
    /*     return; */

    /* std::size_t chunk = 0; */
    /* if (mPrefs.chunkSize) */
    /* { */
    /*     chunk = (mCurrentFrameNumber - mStartFrameNumber) / mPrefs.chunkSize; */
    /*     spdlog::debug("GUI: manual position changed to ({}, {}) on frame {} [chunk {}]", */
    /*         position.x(), position.y(), mCurrentFrameNumber - 1, chunk); */
    /* } */
    /* else */
    /* { */
    /*     spdlog::debug("GUI: manual position changed to ({}, {}) on frame {}", position.x(), */
    /*         position.x(), position.y(), mCurrentFrameNumber - 1, chunk); */
    /* } */

    /* // put it in queue if it does not exist */
    /* if (std::find(std::begin(mDetectionsQueue), std::end(mDetectionsQueue), chunk) */
    /*     == std::end(mDetectionsQueue)) */
    /* { */
    /*     spdlog::debug("GUI: added chunk {} to queue", chunk); */
    /*     mDetectionsQueue.push_back(chunk); */
    /* } */

    /* mManualUnaries.insert(mCurrentFrameNumber - 1, QtOpencvCore::qpoint2point(position)); */

    /* ui->unaryView->getUnaryScene()->setUnaryQuality( */
    /*     mCurrentFrameNumber - 1, UnaryQuality::Excellent); */
    /* /1* ui->unaryView->getUnaryScene()->update(); *1/ */
    /* statusBar()->showMessage("Manually added unary", statusDelay); */
    /* on_buttonNextFrame_clicked(); */
}

void MainWindow::onBearingChanged(QPointF)
{
    /* spdlog::debug("GUI: manual bearing changed to ({}, {}) on frame {}", position.x(), */
    /*     position.y(), mCurrentFrameNumber - 1); */
    // TODO: implement
}

void MainWindow::onPositionCleared()
{
    /* if (!mUnaries.size()) */
    /*     return; */

    /* std::size_t chunk = 0; */
    /* if (mPrefs.chunkSize) */
    /* { */
    /*     chunk = (mCurrentFrameNumber - mStartFrameNumber) / mPrefs.chunkSize; */
    /*     spdlog::debug( */
    /*         "GUI: manual position cleard on frame {} [chunk {}]", mCurrentFrameNumber - 1, chunk); */
    /* } */
    /* else */
    /*     spdlog::debug("GUI: manual position cleard on frame {}", mCurrentFrameNumber - 1); */

    /* // put it in queue if it does not exist */
    /* if (std::find(std::begin(mDetectionsQueue), std::end(mDetectionsQueue), chunk) */
    /*     == std::end(mDetectionsQueue)) */
    /* { */
    /*     spdlog::debug("GUI: added chunk {} to queue", chunk); */
    /*     mDetectionsQueue.push_back(chunk); */
    /* } */

    /* mManualUnaries.clear(mCurrentFrameNumber - 1); */
    /* showFrame(mCurrentFrameNumber); */
    /* statusBar()->showMessage("Manual unary cleared", statusDelay); */

    /* ui->unaryView->getUnaryScene()->setUnaryQuality( */
    /*     mCurrentFrameNumber - 1, mUnaryQualityValues[mCurrentFrameNumber - 1]); */
    /* ui->unaryView->getUnaryScene()->update(); */
}

void MainWindow::onBearingCleared()
{
    /* if (!mImages.size()) */
    /*     return; */
    /* spdlog::debug("GUI: manual bearing cleared on frame {}", mCurrentFrameNumber - 1); */
    // TODO: implement
}

// this is by value by choice, because it needs to be copied and modifed in-place
void MainWindow::setupUnaryScene(std::vector<double> qualities)
{
    /* mUnaryQualities = qualities; */

    /* auto num = mImages.size(); */
    /* auto scene = ui->unaryView->getUnaryScene(); */
    /* scene->setTotalImages(num); */

    /* auto offset = mStartFrameNumber - 1; */
    /* num = mEndFrameNumber - mStartFrameNumber; */
    /* std::size_t medSize = 100; */
    /* mBar->setTotal(num); */
    /* for (std::size_t i = 0; i < std::ceil(static_cast<double>(num) / medSize); i++) */
    /* { */
    /*     auto start = i * medSize + offset; */
    /*     auto end = std::min((i + 1) * medSize, num) + offset; */
    /*     auto mid = start + (end - start) / 2; */
    /*     std::nth_element(std::begin(qualities) + start, std::begin(qualities) + mid, */
    /*         std::begin(qualities) + end); */
    /*     auto median = qualities[mid]; */

    /*     std::vector<double> dists; */
    /*     dists.reserve(end - start); */
    /*     for (std::size_t j = start; j < end; j++) */
    /*         dists.push_back(std::abs(mUnaryQualities[j] - median)); */

    /*     std::nth_element(std::begin(dists), std::begin(dists) + dists.size() / 2, std::end(dists)); */
    /*     auto medianDist = dists[dists.size() / 2]; */
    /*     /1* spdlog::debug("GUI: Unary low threshold {}", median + 2.0 * medianDist); *1/ */
    /*     /1* spdlog::debug("GUI: Unary high threshold {}", median + 4.0 * medianDist); *1/ */
    /*     for (std::size_t j = start; j < end; j++) */
    /*     { */
    /*         UnaryQuality qual; */
    /*         if (mUnaryQualities[j] < (median + 1.5 * medianDist)) */
    /*             qual = UnaryQuality::Good; */
    /*         else if (mUnaryQualities[j] < (median + 3.0 * medianDist)) */
    /*             qual = UnaryQuality::Poor; */
    /*         else */
    /*             qual = UnaryQuality::Critical; */

    /*         scene->setUnaryQuality(j, qual); */
    /*         mUnaryQualityValues[j] = qual; */
    /*         mBar->inc(); */
    /*     } */
    /* } */
    /* mBar->done(); */

    /* // this only works for continous unaries (which should always be the case when using the gui) */
    /* // set all cunks on not-computing */
    /* toggleChunkUnaryScene(-1, false); */

    /* for (const auto& manualUnary : std::as_const(mManualUnaries)) */
    /*     scene->setUnaryQuality(manualUnary.first, UnaryQuality::Excellent); */
}

void MainWindow::toggleChunkUnaryScene(int chunkId, bool computing)
{
    /* auto numUnaries = mUnaries.size(); */
    /* auto chunkSize = mPrefs.chunkSize; */
    /* auto numChunks = Tracker::getNumChunks(numUnaries, chunkSize); */

    /* std::vector<int> chunks; */
    /* if (chunkId == -1) */
    /* { */
    /*     chunks.resize(numChunks); */
    /*     std::iota(std::begin(chunks), std::end(chunks), 0); */
    /* } */
    /* else */
    /*     chunks = {chunkId}; */

    /* for (auto chunk : chunks) */
    /* { */
    /*     UnaryState state; */
    /*     if (computing) */
    /*         state = (chunk % 2) ? UnaryState::ComputingAlt : UnaryState::Computing; */
    /*     else */
    /*         state = (chunk % 2) ? UnaryState::DefaultAlt : UnaryState::Default; */

    /*     auto end = Tracker::getChunkEnd(chunk, numChunks, chunkSize, numUnaries); */
    /*     for (std::size_t j = chunk * chunkSize; j < end; j++) */
    /*         ui->unaryView->getUnaryScene()->setUnaryState(j + mStartFrameNumber - 1, state); */
    /* } */

    /* ui->unaryView->getUnaryScene()->update(); */
}

} // namespace gui
