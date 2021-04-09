#include "panoWizard.h"
#include "ui_panoWizard.h"

#include <QFileDialog>
#include <QtConcurrent>
#include <algorithm>
#include <spdlog/spdlog.h>

#include "habitrack/resultsIO.h"

#include "image-processing/features.h"
#include "image-processing/images.h"
#include "image-processing/matches.h"
#include "image-processing/mildRecommender.h"
#include "image-processing/superGlue.h"
#include "panorama/keyFrameRecommender.h"
#include "panorama/keyFrames.h"
#include "panorama/panoramaStitcher.h"

namespace fs = std::filesystem;
auto ORB = ht::FeatureType::ORB;
auto SIM = ht::GeometricType::Similarity;

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
    spdlog::critical("id: {}, title: {}", id, title.toStdString());
    if (title == "pageProgress")
        this->button(QWizard::BackButton)->setEnabled(false);
    else
        return;

    QtConcurrent::run(this, &PanoWizard::process);
}

void PanoWizard::on_processEvent(int id)
{
    ui->labelVideo->setText(QString("Video: ") + QString::number(id + 1) + QString("/")
        + QString::number(mResFiles.size()));
}

void PanoWizard::process()
{
    for (std::size_t i = 0; i < mResFiles.size(); i++)
    {
        emit processEvent(i);
        processSingle(mResFiles[i]);
    }
}

void PanoWizard::processSingle(const fs::path& resFile)
{
    auto force = ui->checkForce->isChecked();
    auto stage = ui->comboStage->currentIndex();
    int cacheSize = ui->spinCache->value();
    int rows = ui->spinHeight->value();
    int cols = ui->spinWidth->value();
    int numFts = ui->spinNumFts->value();
    auto ftType = FeatureType::ORB;
    if (ui->comboFt->currentText() == "SIFT")
        ftType = FeatureType::SIFT;
    else if (ui->comboFt->currentText() == "SuperPoint")
        ftType = FeatureType::SuperPoint;

    auto resTuple = loadResults(resFile);
    auto imgFolder = std::get<1>(resTuple);

    auto start = std::get<2>(resTuple);
    auto end = std::get<3>(resTuple);
    auto images = Images(imgFolder);
    images.clip(start, end + 1);

    auto basePath = resFile.parent_path() / "panorama";
    fs::create_directories(basePath);

    // calc features for all frames
    auto ftPath = basePath / "fts";
    auto features = Features();
    if (Features::isComputed(images, ftPath, ORB) && !force)
        features = Features::fromDir(images, ftPath, ORB);
    else
        features = Features::compute(images, ftPath, ORB, numFts, cacheSize, size_t_vec(), mBar);

    if (stage < 1)
        return;

    // select key frames
    auto kfPath = basePath / "key_frames.yml";
    std::vector<std::size_t> keyFrames;
    if (KeyFrames::isComputed(kfPath) && !force)
        keyFrames = KeyFrames::fromDir(kfPath);
    else
        keyFrames = KeyFrames::compute(features, SIM, kfPath, 0.3, 0.5, mBar);

    if (stage < 2)
        return;

    // calculate matches between keyframes and intermediate frames via KeyFrameRecommender
    auto kfIntraPath = basePath / "kfs/maches_intra";
    auto kfRecommender = std::make_unique<KeyFrameRecommender>(keyFrames);
    if (!matches::isComputed(kfIntraPath, SIM) || force)
    {
        matches::compute(kfIntraPath, SIM, features, matches::MatchType::Strategy, 0, 0.0,
            std::move(kfRecommender), cacheSize, size_t_vec(), mBar);
    }

    // calculate matches between keyframes via exhaustive matching
    auto kfInterPath = basePath / "kfs/maches_inter";
    auto featuresDensePath = basePath / "kfs/fts";

    auto featuresDense = Features();
    if (Features::isComputed(images, featuresDensePath, ORB, keyFrames) && !force)
        featuresDense = Features::fromDir(images, featuresDensePath, ORB, keyFrames);
    else
    {
        featuresDense = Features::compute(
            images, featuresDensePath, ORB, 4 * numFts, cacheSize, keyFrames, mBar);
    }

    // window is 4 because ceil(1 / 0.3) seems like a sensible default
    auto mildRecommender = std::make_unique<MildRecommender>(featuresDense, 1, true);
    if (ftType == FeatureType::SuperPoint)
    {
        if (Features::isComputed(images, featuresDensePath, ftType, keyFrames) && !force)
        {
            featuresDense = Features::fromDir(images, featuresDensePath, ftType, keyFrames);
        }
        else
        {
            featuresDense = matches::SuperGlue("/data/arbeit/sg/indoor", 800)
                                .compute(images, featuresDensePath, kfInterPath, SIM,
                                    matches::MatchType::Strategy, 4, 0.0,
                                    std::move(mildRecommender), cacheSize, keyFrames, mBar);
        }
    }
    else if (ftType == FeatureType::SIFT)
    {
        auto featuresSift = Features();
        if (Features::isComputed(images, featuresDensePath, ftType, keyFrames) && !force)
        {
            featuresSift = Features::fromDir(images, featuresDensePath, ftType, keyFrames);
        }
        else
        {
            featuresSift = Features::compute(
                images, featuresDensePath, ftType, 4 * numFts, cacheSize, keyFrames, mBar);

            if (!matches::isComputed(kfInterPath, SIM) || force)
            {
                matches::compute(kfInterPath, SIM, featuresSift, matches::MatchType::Strategy, 4,
                    0.0, std::move(mildRecommender), cacheSize, keyFrames, mBar);
            }
        }
        featuresDense = std::move(featuresSift);
    }
    else
    {
        if (!matches::isComputed(kfInterPath, SIM) || force)
        {
            matches::compute(kfInterPath, SIM, featuresDense, matches::MatchType::Strategy, 4, 0.0,
                std::move(mildRecommender), cacheSize, keyFrames, mBar);
        }
    }

    // panoramas can only be calculated from theses Types
    GeometricType useableTypes = matches::getConnectedTypes(kfInterPath, SIM, keyFrames);
    auto typeList = typeToTypeList(useableTypes);
    for (auto type : typeList)
        spdlog::info("Usable type for PanoramaSticher: {}", type);

    if (stage < 3)
        return;

    auto stitcher = PanoramaStitcher(images, keyFrames, SIM);

    // init trafos of keyframes by concatenating them
    stitcher.initTrafos(matches::getTrafos(kfInterPath, SIM));
    auto pano = std::get<0>(stitcher.stitchPano(cv::Size(cols, rows), false, {}, mBar));
    cv::imwrite((basePath / "pano0.png").string(), pano);

    if (stage < 4)
        return;

    // globally optimized these keyframe tranformations and write them for later IVLC
    stitcher.globalOptimizeKeyFrames(featuresDense, matches::getMatches(kfInterPath, SIM), 0, mBar);
    stitcher.writeTrafos(basePath / "kfs/opt_trafos.bin");
    pano = std::get<0>(stitcher.stitchPano(cv::Size(cols, rows), false, {}, mBar));
    cv::imwrite((basePath / "pano1.png").string(), pano);

    if (stage < 5)
        return;

    // reintegrate by geodesic interpolation of frames between keyframes
    stitcher.reintegrate();
    pano = std::get<0>(
        stitcher.stitchPano(cv::Size(cols, rows), false, basePath / "reint_centers.yml", mBar));
    cv::imwrite((basePath / "pano2.png").string(), pano);
    stitcher.writeTrafos(basePath / "reint_trafos.yml", WriteType::Readable);

    if (stage < 6)
        return;

    // refine all keyframes
    stitcher.refineNonKeyFrames(features, matches::getMatches(kfIntraPath, SIM), 50, mBar);
    stitcher.writeTrafos(basePath / "opt_trafos.bin");
    pano = std::get<0>(
        stitcher.stitchPano(cv::Size(cols, rows), false, basePath / "opt_centers.yml", mBar));
    cv::imwrite((basePath / "pano3.png").string(), pano);

    stitcher.writeTrafos(basePath / "opt_trafos.yml", WriteType::Readable);
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

} // namespace gui
