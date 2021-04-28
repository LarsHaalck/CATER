#include "panoWizard.h"
#include "ui_panoWizard.h"

#include <QFileDialog>
#include <QtConcurrent>
#include <algorithm>
#include <spdlog/spdlog.h>

#include "habitrack/resultsIO.h"
#include "image-processing/images.h"
#include "panorama/panoramaEngine.h"
#include "tracker/detections.h"

namespace fs = std::filesystem;

using namespace ht;

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
        QFuture<void> future = QtConcurrent::run(this, &PanoWizard::process);
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
    for (std::size_t i = 0; i < mResFiles.size(); i++)
    {
        emit processEvent(i);
        processSingle(mResFiles[i]);
    }

    if (mResFiles.size() > 1)
    {
        emit processEvent(-1);
        processMultiple();
    }
}

void PanoWizard::populateDefaults()
{
    PanoramaSettings settings;
    ui->spinHeight->setValue(settings.rows);
    ui->spinWidth->setValue(settings.cols);
    ui->spinCache->setValue(settings.cacheSize);

    ui->comboFt->setCurrentIndex(static_cast<int>(settings.ftType));
    ui->spinNumFts->setValue(settings.numFts);
    ui->spinCoverage->setValue(settings.minCoverage);
    ui->checkForce->setChecked(settings.force);
    ui->comboStage->setCurrentIndex(static_cast<int>(settings.stage));

    ui->comboOverlay->setCurrentIndex((settings.overlayCenters << 1) | settings.overlayPoints);
    ui->checkSmooth->setChecked(settings.smooth);
}

PanoramaSettings PanoWizard::getSettings() const
{
    int rows = ui->spinHeight->value();
    int cols = ui->spinWidth->value();
    int cacheSize = ui->spinCache->value();

    auto ftType = static_cast<FeatureType>(ui->comboFt->currentIndex());
    int numFts = ui->spinNumFts->value();
    double coverage = ui->spinCoverage->value();
    auto force = ui->checkForce->isChecked();
    auto stage = ui->comboStage->currentIndex();

    auto overlay = ui->comboOverlay->currentText().toStdString();
    auto overlayCenters = (overlay.find("Centers") != std::string::npos);
    auto overlayPoints = (overlay.find("Detections") != std::string::npos);
    auto smooth = ui->checkSmooth->isChecked();

    PanoramaSettings settings;
    settings.rows = rows;
    settings.cols = cols;
    settings.cacheSize = cacheSize;

    settings.ftType = ftType;
    settings.numFts = numFts;
    settings.minCoverage = coverage;
    settings.force = force;
    settings.stage = static_cast<PanoramaStage>(stage);

    settings.overlayCenters = overlayCenters;
    settings.overlayPoints = overlayPoints;
    settings.smooth = smooth;

    return settings;
}

void PanoWizard::processSingle(const fs::path& resFile)
{
    PanoramaSettings settings = getSettings();
    auto resTuple = loadResults(resFile);
    auto imgFolder = std::get<1>(resTuple);

    auto start = std::get<2>(resTuple);
    auto end = std::get<3>(resTuple);

    Images images(imgFolder);
    images.clip(start, end);

    std::vector<cv::Point> pts;
    if (settings.overlayCenters)
        pts = getDetections(resFile);

    PanoramaEngine::runSingle(images, resFile.parent_path() / "panorama", settings, pts,
        std::get<0>(resTuple).chunkSize, mBar);
}

void PanoWizard::processMultiple()
{
    PanoramaSettings settings = getSettings();
    std::vector<Images> images;
    std::vector<fs::path> data;

    std::vector<cv::Point> pts;
    std::vector<std::size_t> chunkSizes;
    for (const auto& resFile : mResFiles)
    {
        auto resTuple = loadResults(resFile);
        auto imgFolder = std::get<1>(resTuple);

        auto start = std::get<2>(resTuple);
        auto end = std::get<3>(resTuple);
        Images currImgs(imgFolder);
        currImgs.clip(start, end);

        images.push_back(currImgs);
        data.push_back(resFile.parent_path() / "panorama");

        if (settings.overlayCenters)
        {
            auto currPts = getDetections(resFile);
            pts.insert(std::end(pts), std::begin(currPts), std::end(currPts));
        }

        chunkSizes.push_back(std::get<0>(resTuple).chunkSize);
    }

    auto outFolder = mResFiles[0].parent_path() / "panorama_combined";
    PanoramaEngine::runMulti(images, data, outFolder, settings, pts, chunkSizes, mBar);
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

std::vector<cv::Point> PanoWizard::getDetections(const fs::path& resFile)
{
    auto detectionsFile = resFile.parent_path() / "detections.yml";
    if (fs::is_regular_file(detectionsFile))
    {
        auto dets = Detections::fromDir(detectionsFile);
        auto data = dets.data();
        std::vector<cv::Point> vec(dets.size());
        std::transform(std::begin(data), std::end(data), std::begin(vec),
            [](auto pair) { return pair.second.position; });
        return vec;
    }
    else
        return {};
}
} // namespace gui
