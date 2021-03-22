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
    , mViewer(mHabiTrack)
    , mSaved(true)
    , mBlocked(false)
{
    ui->setupUi(this);

    populateGuiDefaults();

    ui->unaryView->setOptimizationFlags(QGraphicsView::DontSavePainterState
        | QGraphicsView::DontAdjustForAntialiasing | QGraphicsView::IndirectPainting);

    mTrackerScene = ui->graphicsView->getTrackerScene();
    ui->graphicsView->setDragMode(QGraphicsView::ScrollHandDrag);
    ui->graphicsView->setFocusPolicy(Qt::NoFocus);
    mUnaryScene = ui->unaryView->getUnaryScene();

    statusBar()->showMessage("", statusDelay);
    setupProgressBar();

    qRegisterMetaType<stdVecDouble>("stdVecDouble");

    /////////////////////////////////////////////////////////////////////////
    // signal connects (some are done in the ui file)
    /////////////////////////////////////////////////////////////////////////
    connect(ui->unaryView, &UnaryGraphicsView::jumpedToUnary, this,
        [&](std::size_t num) { this->showFrame(num + 1); });

    connect(this->ui->graphicsView, SIGNAL(positionChanged(QPointF)), this,
        SLOT(on_positionChanged(QPointF)));
    connect(this->ui->graphicsView, SIGNAL(bearingChanged(QPointF)), this,
        SLOT(on_bearingChanged(QPointF)));
    connect(this->ui->graphicsView, SIGNAL(positionCleared()), this, SLOT(on_positionCleared()));
    connect(this->ui->graphicsView, SIGNAL(bearingCleared()), this, SLOT(on_bearingCleared()));

    connect(&mAutoSaveTimer, SIGNAL(timeout()), this, SLOT(on_actionSave_Results_triggered()));
    connect(this, SIGNAL(toggleChunk(int, bool)), this, SLOT(on_chunkToggled(int, bool)));
    connect(this, SIGNAL(warn(const QString&)), this, SLOT(on_warn(const QString&)));

    connect(this, SIGNAL(trafosExtracted()), this, SLOT(on_trafosExtracted()));
    connect(this, SIGNAL(unariesExtracted()), this, SLOT(on_unariesExtracted()));
    connect(this, SIGNAL(unaryQualitiesExtracted(const std::vector<double>&)), this,
        SLOT(on_unaryQualitiesExtracted(const std::vector<double>&)));
    connect(this, SIGNAL(detectionsAvailable(int)), this, SLOT(on_detectionsAvailable(int)));
}

MainWindow::~MainWindow()
{
    this->blockSignals(true);
    delete ui;
}

void MainWindow::setupProgressBar()
{
    mBar = std::make_shared<ProgressStatusBar>(this);
    connect(mBar.get(), SIGNAL(totalChanged(int)), this, SLOT(on_totalChanged(int)));
    connect(mBar.get(), SIGNAL(incremented(int)), this, SLOT(on_incremented(int)));
    connect(mBar.get(), SIGNAL(incremented()), this, SLOT(on_incremented()));
    connect(mBar.get(), SIGNAL(isDone()), this, SLOT(on_isDone()));
    connect(mBar.get(), SIGNAL(statusChanged(const QString&)), this,
        SLOT(on_statusChanged(const QString&)));
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

void MainWindow::on_overlayTrackedPosition_toggled(bool) { showFrame(mCurrentFrameNumber); }
void MainWindow::on_overlayBearings_toggled(bool) { showFrame(mCurrentFrameNumber); }
void MainWindow::on_overlayTrajectory_toggled(bool) { showFrame(mCurrentFrameNumber); }
void MainWindow::on_trajectorySpin_valueChanged(int) { showFrame(mCurrentFrameNumber); }

void MainWindow::on_actionExpertMode_toggled(bool value)
{
    spdlog::debug("GUI: Toggled expert mode");
    ui->frameUnary->setVisible(value);
    ui->frameTracking->setVisible(value);
    ui->labelExpertMode->setVisible(value);
}

void MainWindow::on_actionSave_Results_triggered()
{
    spdlog::debug("GUI: Save Results triggered");

    if (!mSaved)
    {
        mHabiTrack.saveResultsFile();
        mSaved = true;
        statusBar()->showMessage("Saved results.", statusDelay);
    }
}

void MainWindow::on_actionLabelEditor_triggered()
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

void MainWindow::on_warn(const QString& msg) { QMessageBox::warning(this, "Warning", msg); }
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
    if (mFrameTimer.elapsed() < 50)
        return;
    mFrameTimer.start();

    auto block = ScopedBlocker {ui->actionPrev_Frame};
    on_buttonPrevFrame_clicked();
}

void MainWindow::on_actionNext_Frame_triggered()
{
    if (mFrameTimer.elapsed() < 50)
        return;
    mFrameTimer.start();

    auto block = ScopedBlocker {ui->actionNext_Frame};
    on_buttonNextFrame_clicked();
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
    mTrackerScene->setPixmap(pixMap);

    // set filename
    ui->labelFileName->setText(QString::fromStdString(mHabiTrack.images().getFileName(idx)));

    if (mHabiTrack.unaries().size())
    {
        auto color = mUnaryScene->getUnaryColor(idx);
        auto quality = UnaryScene::unaryQualityToString(mUnaryScene->getUnaryQuality(idx));
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
    ui->actionPrev_Frame->setEnabled(true);
    ui->actionNext_Frame->setEnabled(true);

    ui->overlayGroup->setEnabled(true);

    ui->actionPreferences->setEnabled(true);
    ui->actionLabelEditor->setEnabled(true);

    ui->buttonTrack->setEnabled(true);
    ui->frameTracking->setEnabled(true);
    for (auto& child : ui->frameTracking->findChildren<QWidget*>())
        child->setEnabled(true);

    ui->buttonStartFrame->setEnabled(true);
    ui->buttonEndFrame->setEnabled(true);

    mCurrentFrameNumber = mHabiTrack.getStartFrame();

    using namespace std::chrono_literals;
    mAutoSaveTimer.start(5min);
    mFrameTimer.start();
}

void MainWindow::on_actionOpenImgFolder_triggered()
{
    spdlog::debug("GUI: Triggered Open image folder");
    QString imgFolderPath = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
        mStartPath, QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (imgFolderPath.isEmpty())
        return;

    mHabiTrack.loadImageFolder(fs::path(imgFolderPath.toStdString()));
    openImagesHelper();
    showFrame(mCurrentFrameNumber);
    ui->graphicsView->zoomToFit();
    mSaved = false;
}

void MainWindow::on_actionOpenImgList_triggered() { emit warn("Not implemented yet"); }

void MainWindow::on_actionOpenResultsFile_triggered()
{
    spdlog::debug("GUI: Triggered Open results file");
    QString resultFile = QFileDialog::getOpenFileName(
        this, tr("Open Image File List"), mStartPath, "YAML (*.yml)");
    if (resultFile.isEmpty())
        return;

    mHabiTrack.loadResultsFile(resultFile.toStdString());
    openImagesHelper();

    enqueue(&MainWindow::loadResults, &MainWindow::on_resultsLoaded);
}

void MainWindow::loadResults()
{
    if (mHabiTrack.matchesComputed())
        emit trafosExtracted();
    if (mHabiTrack.unariesComputed())
    {
        extractUnaryQualities();
        emit unariesExtracted();
    }
    if (mHabiTrack.detectionsComputed())
        emit detectionsAvailable(-1);

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(2s);

}

void MainWindow::on_resultsLoaded()
{
    QMutexLocker locker(&mMutex);
    mBlocked = false;
    showFrame(mCurrentFrameNumber);
    ui->graphicsView->zoomToFit();
}

void MainWindow::on_buttonStartFrame_clicked()
{
    if (mCurrentFrameNumber >= mHabiTrack.getEndFrame())
    {
        emit warn("Start frame cannot be equal or greater than end frame");
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
        emit warn("End frame cannot be equal or smaller than start frame");
        return;
    }
    spdlog::debug("GUI: changed end frame to {}", mCurrentFrameNumber);

    // TODO: some data structures are no longer valid and should be computed again
    mHabiTrack.setEndFrame(mCurrentFrameNumber);
    ui->labelEndFrame->setText(QString::number(mCurrentFrameNumber));
    mSaved = false;
}

void MainWindow::on_buttonTrack_clicked() { enqueue(&MainWindow::track, &MainWindow::on_finished); }

void MainWindow::track()
{
    if (!mHabiTrack.featureLoaded())
        extractFeatures();
    extractTrafos();
    if (!mHabiTrack.unariesLoaded())
        extractUnaries();

    optimizeUnaries();
}

bool MainWindow::checkIfBlocked()
{
    QMutexLocker locker(&mMutex);
    if (mBlocked)
        emit warn("Cannot be started due to running operation");
    return mBlocked;
}

void MainWindow::on_buttonExtractFeatures_clicked()
{
    spdlog::debug("GUI: Extract Features");
    enqueue(&MainWindow::extractFeatures, &MainWindow::on_finished);
}

void MainWindow::extractFeatures() { mHabiTrack.extractFeatures(); }

void MainWindow::on_finished()
{
    QMutexLocker locker(&mMutex);
    mBlocked = false;
}

void MainWindow::on_buttonExtractTrafos_clicked()
{
    spdlog::debug("GUI: Clicked Extract Trafos");
    enqueue(&MainWindow::extractTrafos, &MainWindow::on_finished);
}

void MainWindow::extractTrafos()
{
    if (!mHabiTrack.featureLoaded())
    {
        emit warn("Features need to be computed first");
        return;
    }
    mHabiTrack.extractTrafos();
    emit trafosExtracted();
}

void MainWindow::on_trafosExtracted()
{
    QMutexLocker locker(&mMutex);
    auto size = mHabiTrack.trafos().size();
    ui->labelNumTrafos->setText(QString::number(size));

    if (!mHabiTrack.hasUsableTrafos())
    {
        emit warn(
            "Exracted Transformations not a continous chain, Consider increasing feature points.");
    }
}

void MainWindow::on_buttonExtractUnaries_clicked()
{
    spdlog::debug("GUI: Clicked Extract Unaries");
    enqueue(&MainWindow::extractUnaries, &MainWindow::on_finished);
}

void MainWindow::extractUnaries()
{
    if (!mHabiTrack.matchesComputed())
    {
        emit warn("Transformations need to be computed first");
        return;
    }
    mHabiTrack.extractUnaries();
    extractUnaryQualities();
    emit unariesExtracted();
}

void MainWindow::extractUnaryQualities()
{
    if (!mHabiTrack.unariesLoaded())
    {
        emit warn("Unaries need to be loaded first");
        return;
    }
    auto qualities = mHabiTrack.getUnaryQualities();
    emit unaryQualitiesExtracted(qualities);
}

void MainWindow::on_unaryQualitiesExtracted(const stdVecDouble& qualities)
{
    mUnaryScene->setup(qualities, mHabiTrack.getStartFrame(), mHabiTrack.getEndFrame());
}

void MainWindow::on_unariesExtracted()
{
    QMutexLocker locker(&mMutex);
    mBlocked = false;

    ui->labelNumUnaries->setText(QString::number(mHabiTrack.unaries().size()));

    // set all cunks as not-computing
    emit toggleChunk(-1, false);

    for (const auto& manualUnary : mHabiTrack.manualUnaries())
        mUnaryScene->setUnaryQuality(manualUnary.first, UnaryQuality::Excellent);

    // to update unary quality label
    mViewer.reset();
    showFrame(mCurrentFrameNumber);
}

void MainWindow::on_chunkToggled(int chunk, bool compute)
{
    ui->unaryView->getUnaryScene()->toggleChunk(
        chunk, compute, mHabiTrack.getPreferences().chunkSize, mHabiTrack.getStartFrame());
}

void MainWindow::on_buttonOptimizeUnaries_clicked()
{
    spdlog::debug("GUI: Clicked Optimize Unaries");
    if (checkIfBlocked())
        return;

    mBlocked = true;
    optimizeUnaries();
}

void MainWindow::optimizeUnaries()
{
    {
        QMutexLocker locker(&mMutex);

        if (!mHabiTrack.unariesLoaded())
        {
            emit warn("Unaries need to be computed first");
            return;
        }

        // if mDetections empty, then remove all elements form queue and to -1
        int chunk;
        if (!mHabiTrack.detections().size())
        {
            mDetectionsQueue = {};
            chunk = -1;
        }
        else if (mDetectionsQueue.empty())
        {
            emit warn("No new manual unaries supplied. Nothing to do.");
            return;
        }
        else
        {
            chunk = mDetectionsQueue.front();
            mDetectionsQueue.pop_front();
        }
        auto detectionFuture = QtConcurrent::run(&mHabiTrack, &HabiTrack::optimizeUnaries, chunk);
        auto watcher = std::make_unique<QFutureWatcher<void>>();
        watcher->setFuture(detectionFuture);

        connect(watcher.get(), &QFutureWatcher<Detections>::finished, this,
            [&, chunk]() { emit detectionsAvailable(chunk); });
        mDetectionsWatchers[chunk] = std::move(watcher);
        emit toggleChunk(chunk, true);
    }

    if (!mDetectionsQueue.empty())
        optimizeUnaries();
}

void MainWindow::on_detectionsAvailable(int chunkId)
{
    QMutexLocker locker(&mMutex);

#ifdef WITH_QT5_MULTIMEDIA
    QSound::play("qrc:///sounds/notification.wav");
#endif

    statusBar()->showMessage("New detections available", statusDelay);
    emit toggleChunk(chunkId, false);
    spdlog::debug("GUI: detections avaiable for chunk {}", chunkId);
    mDetectionsWatchers.erase(chunkId);

    ui->labelNumTrackedPos->setText(QString::number(mHabiTrack.detections().size()));

    if (mDetectionsQueue.empty())
        mBlocked = false;
}

void MainWindow::on_totalChanged(int total)
{
    ui->progressBar->setValue(0);
    ui->progressBar->setMaximum(total);
}

void MainWindow::on_incremented() { ui->progressBar->setValue(ui->progressBar->value() + 1); }

void MainWindow::on_incremented(int inc)
{
    ui->progressBar->setValue(ui->progressBar->value() + inc);
}

void MainWindow::on_isDone()
{
    ui->labelProgress->setText("Finished");
    ui->progressBar->setValue(ui->progressBar->maximum());
}

void MainWindow::on_statusChanged(const QString& state) { ui->labelProgress->setText(state); }

void MainWindow::on_positionChanged(QPointF position)
{
    if (!mHabiTrack.unaries().size())
        return;

    std::size_t chunkSize = mHabiTrack.getPreferences().chunkSize;
    std::size_t start = mHabiTrack.getStartFrame();
    std::size_t chunk = 0;
    if (chunkSize)
    {
        chunk = (mCurrentFrameNumber - start) / chunkSize;
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

    mHabiTrack.addManualUnary(mCurrentFrameNumber - 1, QtOpencvCore::qpoint2point(position));

    ui->unaryView->getUnaryScene()->setUnaryQuality(
        mCurrentFrameNumber - 1, UnaryQuality::Excellent);
    statusBar()->showMessage("Manually added unary", statusDelay);
    on_buttonNextFrame_clicked();
}

void MainWindow::on_bearingChanged(QPointF)
{
    /* spdlog::debug("GUI: manual bearing changed to ({}, {}) on frame {}", position.x(), */
    /*     position.y(), mCurrentFrameNumber - 1); */
    // TODO: implement
}

void MainWindow::on_positionCleared()
{
    if (!mHabiTrack.unaries().size())
        return;

    std::size_t chunkSize = mHabiTrack.getPreferences().chunkSize;
    std::size_t start = mHabiTrack.getStartFrame();
    std::size_t chunk = 0;
    if (chunkSize)
    {
        chunk = (mCurrentFrameNumber - start) / chunkSize;
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

    mHabiTrack.removeManualUnary(mCurrentFrameNumber - 1);
    showFrame(mCurrentFrameNumber);
    statusBar()->showMessage("Manual unary cleared", statusDelay);
    ui->unaryView->getUnaryScene()->resetUnaryQuality(mCurrentFrameNumber - 1);
}

void MainWindow::on_bearingCleared()
{
    /* if (!mImages.size()) */
    /*     return; */
    /* spdlog::debug("GUI: manual bearing cleared on frame {}", mCurrentFrameNumber - 1); */
    // TODO: implement
}

void MainWindow::on_actionQuit_triggered() { this->close(); }

void MainWindow::closeEvent(QCloseEvent* event)
{
    if (mSaved)
    {
        event->accept();
        return;
    }

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