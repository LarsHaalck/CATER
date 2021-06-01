#include "mainWindow.h"
#include "ui_mainWindow.h"

#include "image-processing/util.h"
#include "labelEditor.h"
#include "labeler.h"
#include "preferencesDialog.h"
#include "qtOpencvCore.h"
#include "scopedBlocker.h"
#include "ui_exportDialog.h"
#include <QDate>
#include <QFileDialog>
#include <QKeyEvent>
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

constexpr int statusDelay = 2000;
constexpr int nextInterval = 50;

namespace gui
{

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , mSaved(true)
    , mLabelsSaved(true)
    , mHabiTrack()
    , mViewer(mHabiTrack)
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
        [&](std::size_t num) { this->showFrame(num); });

    connect(this->ui->graphicsView, SIGNAL(positionChanged(QPointF)), this,
        SLOT(on_positionChanged(QPointF)));
    connect(this->ui->graphicsView, SIGNAL(bearingChanged(QPointF)), this,
        SLOT(on_bearingChanged(QPointF)));
    connect(this->ui->graphicsView, SIGNAL(positionCleared()), this, SLOT(on_positionCleared()));
    connect(this->ui->graphicsView, SIGNAL(bearingCleared()), this, SLOT(on_bearingCleared()));

    connect(&mAutoSaveTimer, &QTimer::timeout, this, [this]() { saveResults(false); });
    connect(this, SIGNAL(toggleChunk(int, bool)), this, SLOT(on_chunkToggled(int, bool)));
    connect(this, SIGNAL(warn(const QString&)), this, SLOT(on_warn(const QString&)));

    connect(this, SIGNAL(trafosExtracted()), this, SLOT(on_trafosExtracted()));
    connect(this, SIGNAL(unariesExtracted()), this, SLOT(on_unariesExtracted()));
    connect(this, SIGNAL(unaryQualitiesExtracted(const std::vector<double>&)), this,
        SLOT(on_unaryQualitiesExtracted(const std::vector<double>&)));
    connect(this, SIGNAL(detectionsAvailable(int)), this, SLOT(on_detectionsAvailable(int)));
    connect(this, SIGNAL(saveResults(bool)), this, SLOT(on_saveResults(bool)));
    connect(this, SIGNAL(breakingBoundaryChange()), this, SLOT(on_breakingBoundaryChange()));
}

MainWindow::~MainWindow()
{
    this->blockSignals(true);
    this->mBar->blockSignals(true);
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
    emit saveResults(true);
}

void MainWindow::on_saveResults(bool force)
{
    if (!mSaved || force)
    {
        mHabiTrack.saveResultsFile();
        mSaved = true;
        statusBar()->showMessage("Saved results.", statusDelay);
    }

    if (!mLabelsSaved || force)
    {
        saveLabelGroupConfigs(mHabiTrack.getOutputPath() / "label_config.json", mLabelConfigs);
        mLabeler.save(mHabiTrack.getOutputPath() / "labels.json");
        mLabelsSaved = true;
        statusBar()->showMessage("Saved labels.", statusDelay);
    }
}

void MainWindow::on_actionLabelEditor_triggered()
{
    LabelEditor editor(this);
    if (!mLabelConfigs.empty())
        editor.setLabelConfigs(mLabelConfigs);
    editor.exec();

    mLabelConfigs = editor.getLabelConfigs();

    mLabeler.init(mHabiTrack.images().size(), mLabelConfigs);
    mLabeler.initDefaultLabels();
    mLabelsSaved = false;
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

        emit saveResults(true);
    }
}

void MainWindow::on_actionExport_triggered()
{
    if (!mHabiTrack.detections().size())
    {
        emit warn("No detections to export yet");
        return;
    }

    QDialog dialog;
    Ui::ExportDialog uiDialog;
    uiDialog.setupUi(&dialog);

    connect(uiDialog.buttonFile, &QPushButton::clicked, this, [&dialog, &uiDialog]() {
        auto filename = QFileDialog::getSaveFileName(
            &dialog, "Export file for detections", QString(), "CSV (*.csv)");
        uiDialog.labelFile->setText(filename);
    });
    dialog.exec();

    mHabiTrack.exportDetections(
        uiDialog.labelFile->text().toStdString(), uiDialog.checkBoxSmooth->isChecked());
}

void MainWindow::on_actionExport_Video_triggered()
{
    mVideoFile = QFileDialog::getSaveFileName(this, tr("Video File"), mStartPath, "MP4 (*.mp4)");
    if (mVideoFile.isEmpty())
        return;
    enqueue(&MainWindow::generateVideo, &MainWindow::on_finished);
}

void MainWindow::generateVideo()
{
    if (!mHabiTrack.detections().size())
    {
        emit warn("No detections to export yet");
        return;
    }

    mHabiTrack.generateVideo(mVideoFile.toStdString());
}

void MainWindow::on_warn(const QString& msg) { QMessageBox::warning(this, "Warning", msg); }
void MainWindow::on_sliderFrame_valueChanged(int value) { showFrame(value - 1); }
void MainWindow::on_spinCurrentFrame_valueChanged(int value) { showFrame(value - 1); }

void MainWindow::on_buttonPrevFrame_clicked()
{
    if (mCurrentFrameNumber > 0)
        showFrame(mCurrentFrameNumber - 1);
}

void MainWindow::on_buttonNextFrame_clicked()
{
    if (mCurrentFrameNumber < mHabiTrack.images().size() - 1)
        showFrame(mCurrentFrameNumber + 1);
}

void MainWindow::on_actionPrev_Frame_triggered()
{
    if (mFrameTimer.elapsed() < nextInterval)
        return;
    mFrameTimer.start();

    auto block = ScopedBlocker {ui->actionPrev_Frame};
    on_buttonPrevFrame_clicked();
}

void MainWindow::on_actionNext_Frame_triggered()
{
    if (mFrameTimer.elapsed() < nextInterval)
        return;
    mFrameTimer.start();

    auto block = ScopedBlocker {ui->actionNext_Frame};
    on_buttonNextFrame_clicked();
}

void MainWindow::updateLabels()
{
    auto labels = mLabeler.getLabels(mCurrentFrameNumber);
    auto sticky = mLabeler.getStickyLabels();
    ui->labelLabel->setText(labels.first);
    ui->labelLabel->setToolTip(labels.second);
    ui->labelSticky->setText(sticky.first);
    ui->labelSticky->setToolTip(sticky.second);
}

void MainWindow::updateSlider()
{
    auto blocker = ScopedBlocker {ui->sliderFrame, ui->spinCurrentFrame};
    ui->sliderFrame->setValue(mCurrentFrameNumber + 1);
    ui->spinCurrentFrame->setValue(mCurrentFrameNumber + 1);
}

void MainWindow::showFrame(std::size_t frame)
{
    mCurrentFrameNumber = frame;
    ht::ImageViewer::VisSettings settings;
    settings.unary = ui->sliderOverlayUnaries->value();
    settings.detection = ui->overlayTrackedPosition->isChecked();
    settings.bearing = ui->overlayBearings->isChecked();
    settings.trajectory = ui->overlayTrajectory->isChecked();
    settings.trajectoryLength = ui->trajectorySpin->value();
    settings.radius = mHabiTrack.getPreferences().detectionRadius;

    auto img = mViewer.getFrame(frame, settings);
    auto pixMap = QPixmap::fromImage(QtOpencvCore::img2qimgRaw(img));
    mTrackerScene->setPixmap(pixMap);

    // set filename
    ui->labelFileName->setText(QString::fromStdString(mHabiTrack.images().getFileName(frame)));

    if (mHabiTrack.unaries().size())
    {
        auto color = mUnaryScene->getUnaryColor(frame);
        auto quality = UnaryScene::unaryQualityToString(mUnaryScene->getUnaryQuality(frame));
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
    mLabeler.processSticky(mCurrentFrameNumber);
    updateLabels();
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

    ui->labelStartFrame->setText(QString::number(mHabiTrack.getStartFrame() + 1));
    ui->labelEndFrame->setText(QString::number(mHabiTrack.getEndFrame() + 1));
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
    ui->actionExport->setEnabled(true);
    ui->actionExport_Video->setEnabled(true);

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
    if (!mHabiTrack.images().size())
    {
        emit warn("No images were loaded. The selected folder contained no image files");
        return;
    }

    openImagesHelper();
    showFrame(mCurrentFrameNumber);
    ui->graphicsView->zoomToFit();
    mSaved = false;
}

void MainWindow::on_actionOpenImgList_triggered() { emit warn("Not implemented yet"); }

void MainWindow::on_actionOpenResultsFile_triggered()
{
    spdlog::debug("GUI: Triggered Open results file");
    QString resultFile
        = QFileDialog::getOpenFileName(this, tr("Open Results"), mStartPath, "YAML (*.yml)");
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
}

bool MainWindow::askContinue(const QString& title, const QString& text)
{
    auto confirm = QMessageBox::question(this, title, text);
    if (confirm == QMessageBox::No)
        return false;
    return true;
}

void MainWindow::on_resultsLoaded()
{
    mBlocked = false;
    ui->graphicsView->zoomToFit();
    showFrame(mCurrentFrameNumber);

    if (!askContinue(
            "Load label config?", "Do you also want load (copy) a label configuration file?"))
        return;

    QString configFile = QFileDialog::getOpenFileName(this,
        tr("Open Label Config File (label_config.json)"), mStartPath, "Label-Config (*.json)");
    if (configFile.isEmpty())
        return;

    mLabelConfigs = loadLabelGroupConfigs(configFile.toStdString());
    mLabeler.init(mHabiTrack.images().size(), mLabelConfigs);
    mLabeler.initDefaultLabels();

    auto labelFile = mHabiTrack.getOutputPath() / "labels.json";
    if (fs::exists(labelFile) && fs::is_regular_file(labelFile))
        mLabeler.load(labelFile);

    updateLabels();
    mLabelsSaved = false;
}

void MainWindow::on_buttonStartFrame_clicked()
{
    if (mCurrentFrameNumber >= mHabiTrack.getEndFrame())
    {
        emit warn("Start frame cannot be equal or greater than end frame");
        return;
    }
    spdlog::debug("GUI: changed start frame to {}", mCurrentFrameNumber);

    bool breaking = false;
    if (mCurrentFrameNumber < mHabiTrack.getStartFrame())
    {
        if (!askContinue("Breaking Change",
                "Changing the value results in multiple data structures to be invalid. "
                "Do you want to continue?"))
            return;
        breaking = true;
    }

    mHabiTrack.setStartFrame(mCurrentFrameNumber);
    ui->labelStartFrame->setText(QString::number(mCurrentFrameNumber + 1));
    mSaved = false;

    if (breaking)
        emit breakingBoundaryChange();
}

void MainWindow::on_buttonEndFrame_clicked()
{
    if (mCurrentFrameNumber <= mHabiTrack.getStartFrame())
    {
        emit warn("End frame cannot be equal or smaller than start frame");
        return;
    }
    spdlog::debug("GUI: changed end frame to {}", mCurrentFrameNumber);

    bool breaking = false;
    if (mCurrentFrameNumber > mHabiTrack.getEndFrame())
    {
        if (!askContinue("Breaking Change",
                "Changing the value results in multiple data structures to be invalid."
                "Do you want to continue?"))
            return;
        breaking = true;
    }

    mHabiTrack.setEndFrame(mCurrentFrameNumber);
    ui->labelEndFrame->setText(QString::number(mCurrentFrameNumber + 1));
    mSaved = false;

    if (breaking)
        emit breakingBoundaryChange();
}

void MainWindow::on_buttonTrack_clicked()
{
    if (!checkIfOptimzing())
        enqueue(&MainWindow::track, &MainWindow::on_tracked);
    else
        on_tracked();
}

void MainWindow::track()
{
    if (!mHabiTrack.featureLoaded())
        extractFeatures();
    extractTrafos();
    if (!mHabiTrack.unariesLoaded())
        extractUnaries();
}

void MainWindow::on_tracked()
{
    mBlocked = false;
    optimizeUnaries();
}

bool MainWindow::checkIfBlocked(bool withOptimize)
{
    if (mBlocked || (withOptimize && checkIfOptimzing()))
    {
        emit warn("Cannot be started due to running operation");
        return true;
    }
    return false;
}

bool MainWindow::checkIfOptimzing() { return !mDetectionsWatchers.empty(); }

void MainWindow::on_buttonExtractFeatures_clicked()
{
    spdlog::debug("GUI: Extract Features");
    enqueue(&MainWindow::extractFeatures, &MainWindow::on_finished);
}

void MainWindow::extractFeatures() { mHabiTrack.extractFeatures(); }

void MainWindow::on_finished() { mBlocked = false; }

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
    auto size = mHabiTrack.trafos().size();
    ui->labelNumTrafos->setText(QString::number(size));
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
    emit saveResults(true);

    // check only for blocked variable and allow if only optimizing
    if (checkIfBlocked(false))
        return;

    optimizeUnaries();
}

void MainWindow::on_breakingBoundaryChange()
{
    // unload everything, delete matches
    mHabiTrack.unload(true);
    mUnaryScene->clear();
    openImagesHelper();
}

void MainWindow::optimizeUnaries()
{
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

    if (!mDetectionsQueue.empty())
        optimizeUnaries();
}

void MainWindow::on_detectionsAvailable(int chunkId)
{
    QSound::play("qrc:///sounds/notification.wav");
    statusBar()->showMessage("New detections available", statusDelay);
    emit toggleChunk(chunkId, false);
    spdlog::debug("GUI: detections avaiable for chunk {}", chunkId);
    mDetectionsWatchers.erase(chunkId);

    ui->labelNumTrackedPos->setText(QString::number(mHabiTrack.detections().size()));

    mSaved = false;
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

void MainWindow::enqueueOptimization()
{
    std::size_t chunk = 0;
    if (auto chunkSize = mHabiTrack.getPreferences().chunkSize; chunkSize)
    {
        auto start = mHabiTrack.getStartFrame();
        chunk = (mCurrentFrameNumber - start) / chunkSize;
    }

    // put it in queue if it does not exist
    if (std::find(std::begin(mDetectionsQueue), std::end(mDetectionsQueue), chunk)
        == std::end(mDetectionsQueue))
    {
        spdlog::debug("GUI: added chunk {} to queue", chunk);
        mDetectionsQueue.push_back(chunk);
    }
}

void MainWindow::on_positionChanged(QPointF position)
{
    auto start = mHabiTrack.getStartFrame();
    auto end = mHabiTrack.getEndFrame();
    if (!mHabiTrack.unaries().size() || mCurrentFrameNumber < start || mCurrentFrameNumber > end)
        return;

    if (auto size = mHabiTrack.images().getImgSize(); position.x() < 0 || position.y() < 0
        || position.x() > size.width || position.y() > size.height)
    {
        return;
    }

    spdlog::debug("GUI: manual position changed to ({}, {}) on frame {}", position.x(),
        position.y(), mCurrentFrameNumber);
    mHabiTrack.addManualUnary(mCurrentFrameNumber, QtOpencvCore::qpoint2point(position));
    enqueueOptimization();
    ui->unaryView->getUnaryScene()->setUnaryQuality(mCurrentFrameNumber, UnaryQuality::Excellent);
    statusBar()->showMessage("Manually added unary", statusDelay);
    mSaved = false;
    on_buttonNextFrame_clicked();
}

void MainWindow::on_bearingChanged(QPointF position)
{
    auto start = mHabiTrack.getStartFrame();
    auto end = mHabiTrack.getEndFrame();
    if (!mHabiTrack.detections().size() || mCurrentFrameNumber < start || mCurrentFrameNumber > end)
        return;
    spdlog::debug("GUI: manual bearing changed to ({}, {}) on frame {}", position.x(), position.y(),
        mCurrentFrameNumber);

    mHabiTrack.addManualBearing(mCurrentFrameNumber, QtOpencvCore::qpoint2point(position));
    showFrame(mCurrentFrameNumber);
}

void MainWindow::on_positionCleared()
{
    auto start = mHabiTrack.getStartFrame();
    auto end = mHabiTrack.getEndFrame();
    if (!mHabiTrack.unaries().size() || mCurrentFrameNumber < start || mCurrentFrameNumber > end)
        return;

    spdlog::debug("GUI: manual position cleared on frame {}", mCurrentFrameNumber);
    mHabiTrack.removeManualUnary(mCurrentFrameNumber);
    enqueueOptimization();
    ui->unaryView->getUnaryScene()->resetUnaryQuality(mCurrentFrameNumber);
    statusBar()->showMessage("Manual unary cleared", statusDelay);
    mSaved = false;
    showFrame(mCurrentFrameNumber);
}

void MainWindow::on_bearingCleared()
{
    auto start = mHabiTrack.getStartFrame();
    auto end = mHabiTrack.getEndFrame();
    if (!mHabiTrack.detections().size() || mCurrentFrameNumber < start || mCurrentFrameNumber > end)
        return;
    spdlog::debug("GUI: manual bearing cleared on frame {}", mCurrentFrameNumber);
    mHabiTrack.removeManualBearing(mCurrentFrameNumber);
    showFrame(mCurrentFrameNumber);
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
        emit saveResults(true);
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

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    mLabeler.processKeyEvent(mCurrentFrameNumber, event);
    updateLabels();
}

} // namespace gui
