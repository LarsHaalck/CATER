#include "panoWizard.h"
#include "ui_panoWizard.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QtConcurrent>
#include <algorithm>
#include <spdlog/spdlog.h>

#include <cater/image-processing/images.h>
#include <cater/model/resultsIO.h>
#include <cater/panorama/panoramaEngine.h>
#include <cater/tracker/detections.h>
#include <cater/util/log.h>
#include <cater/util/fmt.h>

namespace fs = std::filesystem;

namespace gui
{
PanoWizard::PanoWizard(QWidget* parent)
    : QWizard(parent)
    , ui(new Ui::PanoWizard)
{
    ui->setupUi(this);

    connect(ui->buttonLoad, SIGNAL(clicked()), this, SLOT(on_load()));
    connect(this, SIGNAL(currentIdChanged(int)), this, SLOT(on_newid(int)));

    mBar = std::make_shared<ProgressStatusBar>(this);
    connect(mBar.get(), SIGNAL(totalChanged(int)), this, SLOT(on_totalChanged(int)));
    connect(mBar.get(), SIGNAL(incremented(int)), this, SLOT(on_incremented(int)));
    connect(mBar.get(), SIGNAL(incremented()), this, SLOT(on_incremented()));
    connect(mBar.get(), SIGNAL(isDone()), this, SLOT(on_isDone()));
    connect(mBar.get(), SIGNAL(statusChanged(const QString&)), this,
        SLOT(on_statusChanged(const QString&)));

    connect(this, SIGNAL(processEvent(int)), this, SLOT(on_processEvent(int)));

    populateDefaults();
}

PanoWizard::~PanoWizard() { delete ui; }

void PanoWizard::on_load()
{
    QString resultFile
        = QFileDialog::getOpenFileName(this, tr("Open Results"), QString(), "YAML (*.yml)");
    if (resultFile.isEmpty())
        return;

    fs::path file = resultFile.toStdString();

    if (!fs::exists(file) || !fs::is_regular_file(file) || file.filename() != "results.yml")
        return;

    if (std::find(std::begin(mResFiles), std::end(mResFiles), file) != std::end(mResFiles))
        return;

    mResFiles.push_back(file);
    ui->listWidget->addItem(resultFile);
}

void PanoWizard::on_newid(int id)
{
    if (id == -1)
        return;

    auto title = this->page(id)->objectName();
    if (title == "pageProgress")
    {
        this->button(QWizard::BackButton)->setEnabled(false);
        this->button(QWizard::FinishButton)->setEnabled(false);
    }
    else
        return;

    if (!mResFiles.empty())
    {
        connect(&mWatcher, &QFutureWatcher<void>::finished,
            [&]() { this->button(QWizard::FinishButton)->setEnabled(true); });
        QFuture<void> future = QtConcurrent::run(&PanoWizard::process, this);
        mWatcher.setFuture(future);
    }
}

void PanoWizard::on_processEvent(int id)
{
    if (id >= 0)
    {
        ui->labelVideo->setText(QString("Video: ") + QString::number(id + 1) + QString("/")
            + QString::number(mResFiles.size()));
    }
    else
        ui->labelVideo->setText("Inter-Video Panorama");
}

void PanoWizard::process()
{
    std::vector<ct::Images> images;
    std::vector<fs::path> data;

    std::vector<cv::Point> pts;
    std::vector<std::size_t> chunkSizes;
    ct::PanoramaSettings settings = getSettings();
    std::vector<ct::GPSInterpolator> gpsInterpolators;
    for (std::size_t i = 0; i < mResFiles.size(); i++)
    {
        auto resFile = mResFiles[i];

        ct::Model cater;
        cater.loadResultsFile(resFile);
        ct::GPSSettings gps_settings;
        auto gpsInterpolator = ct::GPSInterpolator(gps_settings, cater.getStartFrame());
        gpsInterpolators.push_back(gpsInterpolator);

        ct::Images currImages = cater.images();
        currImages.clip(cater.getStartFrame(), cater.getEndFrame() + 1);

        std::vector<cv::Point> currPts;
        if (settings.overlayPoints)
            currPts = getDetections(cater);

        auto outPath = cater.getOutputPath() / "panorama";

        ct::setLogFileTo(outPath / "log.txt");
        ct::PanoramaEngine::runSingle(currImages, outPath, settings, currPts,
            cater.getPreferences().chunkSize, {}, mBar);

        images.push_back(currImages);
        data.push_back(outPath);
        pts.insert(std::end(pts), std::begin(currPts), std::end(currPts));

        chunkSizes.push_back(cater.getPreferences().chunkSize);

        emit processEvent(i);
    }

    if (mResFiles.size() <= 1)
        return;

    emit processEvent(-1);

    auto outFolder = mResFiles[0].parent_path() / "panorama_combined";
    ct::setLogFileTo(outFolder / "log.txt");
    for (auto resFile : mResFiles)
        spdlog::info("Adding res file for combined panorama: {}", resFile);

    try
    {
        ct::PanoramaEngine::runMulti(
            images, data, outFolder, settings, pts, chunkSizes, gpsInterpolators, mBar);
    }
    catch (const std::runtime_error& e)
    {
        QMessageBox::critical(this, "Error", e.what());
    }
}

void PanoWizard::populateDefaults()
{
    ct::PanoramaSettings settings;
    ui->spinHeight->setValue(settings.rows);
    ui->spinWidth->setValue(settings.cols);
    ui->spinCache->setValue(settings.cacheSize);

    ui->comboFt->setCurrentIndex(static_cast<int>(settings.featureType));
    ui->spinNumFts->setValue(settings.numFeatures);
    ui->spinCoverage->setValue(settings.minCoverage);
    ui->writeDebug->setChecked(settings.writeReadable);
    ui->checkForce->setChecked(settings.force);
    ui->comboStage->setCurrentIndex(static_cast<int>(settings.stage));

    ui->comboOverlay->setCurrentIndex((settings.overlayCenters << 1) | settings.overlayPoints);
}

ct::PanoramaSettings PanoWizard::getSettings() const
{
    int rows = ui->spinHeight->value();
    int cols = ui->spinWidth->value();
    int cacheSize = ui->spinCache->value();

    auto ftType = static_cast<ct::FeatureType>(ui->comboFt->currentIndex());
    int numFts = ui->spinNumFts->value();
    double coverage = ui->spinCoverage->value();
    auto writeReadable = ui->writeDebug->isChecked();
    auto force = ui->checkForce->isChecked();
    auto stage = ui->comboStage->currentIndex();

    auto overlay = ui->comboOverlay->currentText().toStdString();
    auto overlayCenters = (overlay.find("Centers") != std::string::npos);
    auto overlayPoints = (overlay.find("Detections") != std::string::npos);

    ct::PanoramaSettings settings;
    settings.rows = rows;
    settings.cols = cols;
    settings.cacheSize = cacheSize;

    settings.featureType = ftType;
    settings.numFeatures = numFts;
    settings.minCoverage = coverage;
    settings.writeReadable = writeReadable;
    settings.force = force;
    settings.stage = static_cast<ct::PanoramaStage>(stage);

    settings.overlayCenters = overlayCenters;
    settings.overlayPoints = overlayPoints;

    return settings;
}


void PanoWizard::on_totalChanged(int total)
{
    ui->progressBar->setValue(0);
    ui->progressBar->setMaximum(total);
}

void PanoWizard::on_incremented() { ui->progressBar->setValue(ui->progressBar->value() + 1); }

void PanoWizard::on_incremented(int inc)
{
    ui->progressBar->setValue(ui->progressBar->value() + inc);
}

void PanoWizard::on_isDone()
{
    ui->labelProgress->setText("Finished");
    ui->progressBar->setValue(ui->progressBar->maximum());
}

void PanoWizard::on_statusChanged(const QString& state) { ui->labelProgress->setText(state); }

std::vector<cv::Point> PanoWizard::getDetections(const ct::Model& cater) const
{
    auto dets = cater.detections();
    auto data = dets.cdata();
    std::vector<cv::Point> vec(dets.size());
    std::transform(std::begin(data), std::end(data), std::begin(vec),
        [](auto pair) { return pair.second.position; });
    // repeat last element to have as many detections as trafos
    vec.push_back(*vec.rbegin());
    return vec;
}
} // namespace gui
