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
    , mViewer(mHabiTrack)
    , mSaved(true)
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

    connect(ui->unaryView, &UnaryGraphicsView::jumpedToUnary, this,
        [=](std::size_t num) { this->showFrame(num + 1); });

    connect(this->ui->graphicsView, SIGNAL(positionChanged(QPointF)), this,
        SLOT(onPositionChanged(QPointF)));
    connect(this->ui->graphicsView, SIGNAL(bearingChanged(QPointF)), this,
        SLOT(onBearingChanged(QPointF)));
    connect(this->ui->graphicsView, SIGNAL(positionCleared()), this, SLOT(onPositionCleared()));
    connect(this->ui->graphicsView, SIGNAL(bearingCleared()), this, SLOT(onBearingCleared()));

    connect(this, SIGNAL(detectionsAvailable(int)), this, SLOT(onDetectionsAvailable(int)));
}

MainWindow::~MainWindow()
{
    this->blockSignals(true);
    delete ui;
}

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
    auto blocker = ScopedBlocker {ui->sliderOverlayUnaries, ui->overlayTrackedPosition,
        ui->overlayBearings, ui->overlayTrajectory, ui->trajectorySpin};
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
    mGuiPrefs.overlayUnaries = value;
    showFrame(mCurrentFrameNumber);
}

void MainWindow::on_overlayTrackedPosition_toggled(bool)
{
    showFrame(mCurrentFrameNumber);
}

void MainWindow::on_overlayBearings_toggled(bool)
{
    showFrame(mCurrentFrameNumber);
}

void MainWindow::on_overlayTrajectory_toggled(bool)
{
    showFrame(mCurrentFrameNumber);
}

void MainWindow::on_trajectorySpin_valueChanged(int)
{
    showFrame(mCurrentFrameNumber);
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
    spdlog::debug("GUI: Save Results triggered");

    mHabiTrack.saveResultsFile();
    mSaved = true;
    statusBar()->showMessage("Saved results.", statusDelay);
}

void MainWindow::on_actionLabel_Editor_triggered()
{
    /* LabelDialog labelDialog(this); */
    /* labelDialog.exec(); */
}

void MainWindow::on_actionPreferences_triggered()
{
    PreferencesDialog prefDialog(this, mHabiTrack.getPreferences());
    auto code = prefDialog.exec();
    if (code == QDialog::Accepted)
    {
        // overwrite current settings if accepted, discard otherwise
        auto prefs = prefDialog.getPreferences();
        spdlog::debug("GUI: Changed Preferences to: {}", prefs);
        mHabiTrack.setPreferences(prefs);

        // TODO: should this be saved implicitly?
        mHabiTrack.saveResultsFile();
        mSaved = true;
    }
}

void MainWindow::on_sliderFrame_valueChanged(int value) { showFrame(value); }

void MainWindow::on_spinCurrentFrame_valueChanged(int value) { showFrame(value); }

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
    auto blocker = ScopedBlocker {ui->sliderFrame, ui->spinCurrentFrame};
    ui->sliderFrame->setValue(mCurrentFrameNumber);
    ui->spinCurrentFrame->setValue(mCurrentFrameNumber);
}

void MainWindow::showFrame(std::size_t frame)
{
    mCurrentFrameNumber = frame;
    auto idx = frame - 1;

    ImageViewer::VisSettings settings;
    settings.unary = ui->sliderOverlayUnaries->value();
    settings.detection = ui->overlayTrackedPosition->isChecked();
    settings.bearing = ui->overlayBearings->isChecked();
    settings.trajectory = ui->overlayTrajectory->isChecked();
    settings.trajectoryLength = ui->trajectorySpin->value();

    auto img = mViewer.getFrame(idx, settings);
    auto pixMap = QPixmap::fromImage(QtOpencvCore::img2qimgRaw(img));
    mScene->setPixmap(pixMap);

    // set filename
    ui->labelFileName->setText(QString::fromStdString(mHabiTrack.images().getFileName(idx)));

    if (mHabiTrack.unaries().size())
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

    ui->overlayGroup->setEnabled(true);

    ui->buttonTrack->setEnabled(true);
    ui->frameTracking->setEnabled(true);
    for (auto& child : ui->frameTracking->findChildren<QWidget*>())
        child->setEnabled(true);

    ui->buttonStartFrame->setEnabled(true);
    ui->buttonEndFrame->setEnabled(true);

    // show first frame
    mCurrentFrameNumber = mHabiTrack.getStartFrame();
    showFrame(mCurrentFrameNumber);
    ui->graphicsView->zoomToFit();
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
    mSaved = false;
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
    openImagesHelper();
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

    mSaved = false;
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

    mSaved = true;
}

void MainWindow::on_mikeButton_clicked()
{
    if (checkRunningThread())
        return;

    on_buttonExtractFeatures_clicked();
    on_buttonExtractTrafos_clicked();
    on_buttonExtractUnaries_clicked();
    on_buttonOptimizeUnaries_clicked();
}

bool MainWindow::checkRunningThread()
{
    if (!mBlockingThread || !mUnaryQualityWatcher)
        return false;

    if (mBlockingThread->isFinished() || mUnaryQualityWatcher->isRunning())
    {
        QMessageBox::warning(this, "Warning", "Wait for current process to finish");
        return true;
    }

    return false;
}

void MainWindow::on_buttonExtractFeatures_clicked()
{
    spdlog::debug("GUI: Clicked Extract Features");
    if (checkRunningThread())
        return;

    mBlockingThread
        = std::unique_ptr<QThread>(QThread::create(&HabiTrack::extractFeatures, &mHabiTrack));
    mBlockingThread->start();
}

void MainWindow::on_buttonExtractTrafos_clicked()
{
    spdlog::debug("GUI: Clicked Extract Trafos");
    if (!mHabiTrack.featureComputed())
    {
        QMessageBox::warning(this, "Warning", "Features need to be computed first");
        return;
    }

    if (checkRunningThread())
        return;

    mBlockingThread
        = std::unique_ptr<QThread>(QThread::create(&HabiTrack::extractTrafos, &mHabiTrack));
    connect(mBlockingThread.get(), SIGNAL(finished()), this, SLOT(on_trafosExtracted()));
    mBlockingThread->start();
}

void MainWindow::on_trafosExtracted()
{
    auto size = mHabiTrack.trafos().size();
    ui->labelNumTrafos->setText(QString::number(size));

    if (!mHabiTrack.hasUsableTrafos())
    {
        QMessageBox::warning(this, "Warning",
            "Exracted Transformations not a continous chain, Consider increasing feature points.");
    }
}

void MainWindow::on_buttonExtractUnaries_clicked()
{
    QMutexLocker locker(&mMutex);
    spdlog::debug("GUI: Clicked Extract Unaries");
    if (!mHabiTrack.matchesComputed())
    {
        QMessageBox::warning(this, "Warning", "Transformations need to be computed first");
        return;
    }

    if (checkRunningThread())
        return;

    mBlockingThread
        = std::unique_ptr<QThread>(QThread::create(&HabiTrack::extractUnaries, &mHabiTrack));
    connect(mBlockingThread.get(), SIGNAL(finished()), this, SLOT(on_unariesExtracted()));
    mBlockingThread->start();
}

void MainWindow::on_unariesExtracted()
{
    mViewer.reset();
    QFuture<std::vector<double>> future
        = QtConcurrent::run(&mHabiTrack, &HabiTrack::getUnaryQualities);
    auto watcher = std::make_unique<QFutureWatcher<std::vector<double>>>();
    watcher->setFuture(future);

    connect(watcher.get(), SIGNAL(finished()), this, SLOT(on_unariesQualitesCalculated()));
    mUnaryQualityWatcher = std::move(watcher);
}

void MainWindow::on_unariesQualitesCalculated()
{
    auto scene = ui->unaryView->getUnaryScene();
    auto unaryQualities = mUnaryQualityWatcher->result();
    scene->setup(unaryQualities, mHabiTrack.getStartFrame(), mHabiTrack.getEndFrame());

    ui->labelNumUnaries->setText(QString::number(mHabiTrack.unaries().size()));

    // set all cunks as not-computing
    toggleChunk(-1, false);

    for (const auto& manualUnary : mHabiTrack.manualUnaries())
        scene->setUnaryQuality(manualUnary.first, UnaryQuality::Excellent);
}

void MainWindow::toggleChunk(int chunk, bool compute)
{
    ui->unaryView->getUnaryScene()->toggleChunk(
        chunk, compute, mHabiTrack.getPreferences().chunkSize, mHabiTrack.getStartFrame());
}

void MainWindow::on_buttonOptimizeUnaries_clicked()
{
    {
        QMutexLocker locker(&mMutex);

        spdlog::debug("GUI: Clicked Optimize Unaries");

        if (!mHabiTrack.unariesComputed())
        {
            QMessageBox::warning(this, "Warning", "Unaries need to be computed first");
            return;
        }

        // check queue
        // if mDetections empty, then remove all elements form queue and to -1
        int chunk;
        if (!mHabiTrack.detections().size())
        {
            mDetectionsQueue = {};
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
            mDetectionsQueue.pop();
        }

        toggleChunk(chunk, true);

        QFuture<void> detectionFuture;
        if (chunk == -1)
            detectionFuture = QtConcurrent::run(&mHabiTrack, &HabiTrack::optimizeUnaries);
        else
            detectionFuture = QtConcurrent::run(&mHabiTrack, &HabiTrack::optimizeUnaries, chunk);

        auto watcher = std::make_unique<QFutureWatcher<void>>();
        watcher->setFuture(detectionFuture);

        connect(watcher.get(), &QFutureWatcher<Detections>::finished, this,
            [this, chunk]() { this->onDetectionsAvailable(chunk); });
        mDetectionsWatchers[chunk] = std::move(watcher);
    }

    if (!mDetectionsQueue.empty())
        on_buttonOptimizeUnaries_clicked();
}

void MainWindow::onDetectionsAvailable(int chunkId)
{
    QMutexLocker locker(&mMutex);

#ifdef WITH_QT5_MULTIMEDIA
    QSound::play("qrc:///sounds/notification.wav");
#endif

    statusBar()->showMessage("New detections available", statusDelay);
    toggleChunk(chunkId, false);
    spdlog::debug("GUI: detections avaiable for chunk {}", chunkId);
    mDetectionsWatchers.erase(chunkId);
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

void MainWindow::onStatusChanged(const QString& state) { ui->labelProgress->setText(state); }

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

void MainWindow::closeEvent(QCloseEvent* event)
{
    QMessageBox msgBox;
    msgBox.setText("You have unsafed changes.");
    msgBox.setInformativeText("Do you want to save?");
    msgBox.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Save);
    int ret = msgBox.exec();

    switch (ret)
    {
    case QMessageBox::Save:
        mHabiTrack.saveResultsFile();
        event->accept();
        break;
    case QMessageBox::Discard:
        event->accept();
        break;
    case QMessageBox::Cancel:
        event->ignore();
        break;
    default:
        break;
    }
}

} // namespace gui
