#include <cater/model/model.h>

#include <cater/image-processing/util.h>
#include <cater/io/io.h>
#include <cater/model/imageViewer.h>
#include <cater/model/resultsIO.h>
#include <cater/progressbar/progressBar.h>
#include <cater/tracker/tracker.h>
#include <cater/util/log.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>
#include <cater/tracker/fmt.h>

namespace fs = std::filesystem;

namespace ct
{
Model::Model()
    : mBar(std::make_shared<ProgressBar>())
{
    setTrackerSettings(mPrefs);
}

bool Model::featureComputed() const
{
    return Features::isComputed(
        mImages, mFtFolder, mPrefs.featureType, mStartFrameNumber, mEndFrameNumber + 1);
}

bool Model::matchesComputed() const
{
    if (mMatchFolder.empty())
        return false;
    return matches::isComputed(mMatchFolder, GeometricType::Homography);
}

bool Model::unariesComputed() const
{
    return Unaries::isComputed(mImages, mUnFolder, mStartFrameNumber, mEndFrameNumber + 1);
}

bool Model::detectionsComputed() const { return fs::is_regular_file(mDetectionsFile); }
bool Model::featureLoaded() const { return mFeatures.size(); }
bool Model::unariesLoaded() const { return mUnaries.size(); }
bool Model::detectionsLoaded() const { return mDetections.size(); }

void Model::unload(bool deleteMatches)
{
    mFeatures = {};
    mUnaries = {};
    mDetections = {};

    if (deleteMatches)
    {
        for (auto& p : fs::directory_iterator(mMatchFolder))
        {
            if (p.is_regular_file())
            {
                auto file = p.path().filename().string();
                if (file.rfind("matches.", 0) == 0 || file.rfind("trafos.", 0) == 0)
                    fs::remove(p);
            }
        }
    }
}

void Model::loadImageFolder(const fs::path& imgFolder)
{
    mImgFolder = fs::absolute(imgFolder);
    mImages = Images(mImgFolder);
    if (!mImages.size())
        return;

    mStartFrameNumber = 0;
    mEndFrameNumber = mImages.size() - 1;
    openImagesHelper();
}

void Model::loadResultsFile(const fs::path& resultFile)
{
    auto parentPath = resultFile.parent_path();
    auto [prefs, imgFolder, start, end] = loadResults(resultFile);
    mPrefs = prefs;
    setTrackerSettings(prefs);
    mImgFolder = imgFolder;
    mStartFrameNumber = start;
    mEndFrameNumber = end;

    mImages = Images(mImgFolder);
    openImagesHelper(parentPath);

    if (featureComputed() || !mPrefs.removeCamMotion)
    {
        if (mPrefs.removeCamMotion)
            extractFeatures();
        if (matchesComputed() || !mPrefs.removeCamMotion)
        {
            if (mPrefs.removeCamMotion)
                extractTrafos();
            if (unariesComputed())
            {
                extractUnaries();
                if (detectionsComputed())
                    mDetections = Detections::fromDir(mDetectionsFile);
            }
        }
    }

    /* debug plots */
    /* auto trafs = trafos(); */
    /* for (int i = 0; i < 150; i++) */
    /* { */
    /*     auto currImg = mImages.at(i); */
    /*     cv::Mat trafo = transformation::getIdentity(true); */
    /*     for (int j = i; j >= 1; j--) */
    /*     { */
    /*         trafo = transformation::invert( */
    /*                 trafs.at({j - 1, j}), GeometricType::Homography) * trafo; */
    /*     } */
    /*     cv::warpPerspective(currImg, currImg, trafo, currImg.size()); */

    /*     std::stringstream filename; */
    /*     filename << std::setw(3) << std::setfill('0') << i; */
    /*     cv::imwrite(std::string("out/") + std::string("wrap_") + filename.str() + ".png", currImg); */
    /* } */
}

void Model::generateVideo(const fs::path& videoFilename) const
{
    auto fps = mPrefs.fps > 0 ? mPrefs.fps : 50;
    cv::VideoWriter video(
        videoFilename, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, mImages.getImgSize());

    if (!video.isOpened())
        spdlog::critical("Could not write video, {} could not be opened", videoFilename);

    ImageViewer::VisSettings settings;
    settings.trajectory = true;
    settings.trajectoryLength = fps;
    settings.showManual = false;
    ImageViewer viewer(*this, ImageViewer::Cache::Disable);

    mBar->status("Exporting Video");
    mBar->setTotal(mEndFrameNumber - mStartFrameNumber);
    for (auto i = mStartFrameNumber; i <= mEndFrameNumber; i++)
    {
        auto frame = viewer.getFrame(i, settings);
        video.write(frame);
        mBar->inc();
    }
    mBar->done();
}

void Model::saveResultsFile()
{
    saveResults(mResultsFile, mPrefs, mImgFolder, mStartFrameNumber, mEndFrameNumber);
    saveManualUnaries();
    saveDetections();
}

void Model::saveDetections(std::optional<fs::path> detectionsFile)
{
    if (mDetections.size())
        mDetections.save(detectionsFile.value_or(mDetectionsFile));
}

void Model::saveManualUnaries()
{
    if (mManualUnaries.size())
        mManualUnaries.save(mUnFolder);
}

void Model::setPreferences(const Preferences& prefs)
{
    mPrefs = prefs;
    setTrackerSettings(prefs);
}
Preferences Model::getPreferences() const { return mPrefs; }

void Model::populatePaths()
{
    spdlog::debug("Model: Generated output path: {}", mOutputPath.string());

    mResultsFile = mOutputPath / "results.yml";
    mFtFolder = mOutputPath / "fts";
    mMatchFolder = mOutputPath / "matches";
    mUnFolder = mOutputPath / "unaries";
    mDetectionsFile = mOutputPath / "detections.yml";

    setLogFileTo(mOutputPath / "logs.txt");
}

void Model::openImagesHelper(const fs::path& path)
{
    if (path.empty())
    {
        auto tempPath = mImgFolder;
        if (fs::is_directory(tempPath))
            mOutputPath = tempPath;
        else
            mOutputPath = tempPath.filename();

        mOutputPath += "_output";

        mOutputPath /= "now";
        /* mOutputPath /= getDateTimeString(); */
    }
    else
        mOutputPath = path;

    populatePaths();
}

std::string Model::getDateTimeString() const
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

void Model::setTrackerSettings(const Preferences& prefs)
{
    Tracker::Settings settings;
    settings.subsample = prefs.unarySubsample;
    settings.pairwiseSize = prefs.pairwiseSize;
    settings.pairwiseSigma = prefs.pairwiseSigma;
    settings.manualMultiplier = prefs.unaryMultiplier;
    settings.smoothBearing = prefs.smoothBearing;
    settings.windowSize = prefs.smoothBearingWindowSize;
    settings.outlierTolerance = prefs.smoothBearingOutlierTol;
    settings.chunkSize = prefs.chunkSize;
    mTrackerSettings = std::move(settings);
}

void Model::setStartFrame(std::size_t frame) { mStartFrameNumber = frame; }
void Model::setEndFrame(std::size_t frame) { mEndFrameNumber = frame; }

void Model::extractFeatures()
{
    spdlog::debug("Model: Clicked Extract Features");

    auto start = mStartFrameNumber;
    auto end = mEndFrameNumber + 1;
    if (featureComputed())
    {
        mFeatures = Features::fromDir(mImages, mFtFolder, mPrefs.featureType, start, end);
        mBar->done();
        return;
    }
    mFeatures = Features::compute(mImages, mFtFolder, mPrefs.featureType, mPrefs.numFeatures, start,
        end, mPrefs.cacheSize, mBar);
}

void Model::extractTrafos()
{
    spdlog::debug("Model: Clicked Extract Trafos");

    if (!featureComputed())
        throw ModelException("Features need to be computed before trafos.");

    if (matchesComputed())
        mBar->done();
    else
    {
        matches::compute(mMatchFolder, GeometricType::Homography, mFeatures,
            matches::MatchType::Windowed, 2, 0.0, nullptr, mPrefs.cacheSize,
            util::getContinuousIds(mStartFrameNumber, mEndFrameNumber + 1), mBar);
    }
}

void Model::extractUnaries()
{
    spdlog::debug("Model: Extract Unaries");

    if (!matchesComputed() && mPrefs.removeCamMotion)
        throw ModelException("Trafos need to be computed before Unaries.");

    auto start = mStartFrameNumber;
    auto end = mEndFrameNumber + 1;
    if (unariesComputed())
    {
        mUnaries = Unaries::fromDir(mImages, mUnFolder, start, end);
        mBar->done();
    }
    else
    {
        auto trafos = mPrefs.removeCamMotion
            ? matches::getTrafos(mMatchFolder, GeometricType::Homography)
            : PairwiseTrafos();

        mUnaries = Unaries::compute(mImages, mUnFolder, start, end, mPrefs.removeRedLasers,
            mPrefs.unarySubsample, mPrefs.unarySigma, trafos, mPrefs.cacheSize, mBar);
    }

    if (mPrefs.ignoreManualUnaries)
    {
        spdlog::warn("Ignoring manual unaries per request");
        mManualUnaries
            = ManualUnaries(mPrefs.unarySubsample, mPrefs.manualUnarySize, mImages.getImgSize());
    }
    else
    {
        // zero init if existent
        mManualUnaries = ManualUnaries::fromDir(
            mUnFolder, mPrefs.unarySubsample, mPrefs.manualUnarySize, mImages.getImgSize());
    }
}

std::vector<double> Model::getUnaryQualities()
{
    mBar->status("Calculating Unary Qualities");
    mBar->setTotal(mEndFrameNumber - mStartFrameNumber);
    std::vector<double> unaryQuality(mImages.size(), -1);
    for (auto i = mStartFrameNumber + 1; i < mEndFrameNumber; i++)
    {
        unaryQuality[i] = Unaries::getUnaryQuality(mUnaries.at(i));
        mBar->inc();
    }
    mBar->done();
    return unaryQuality;
}

bool Model::hasUsableTrafos() const
{
    auto types = ct::matches::getConnectedTypes(mMatchFolder, ct::GeometricType::Homography,
        util::getContinuousIds(mStartFrameNumber, mEndFrameNumber + 1));

    if (static_cast<unsigned int>(types & ct::GeometricType::Homography))
    {
        spdlog::info("Model: Transformations usable for unary extraction.");
        return true;
    }
    else
    {
        spdlog::warn("Model: Exracted Transformations not a continous chain, \
            Consider increasing feature points.");
        return false;
    }
}

PairwiseMatches Model::matches() const
{
    if (matchesComputed())
        return matches::getMatches(mMatchFolder, GeometricType::Homography);
    return PairwiseMatches();
}

PairwiseTrafos Model::trafos() const
{
    if (matchesComputed())
        return matches::getTrafos(mMatchFolder, GeometricType::Homography);
    return PairwiseTrafos();
}

void Model::optimizeUnaries(int chunk)
{
    spdlog::debug("Model: Optimize Unaries (chunk {})", chunk);

    if (mTrafos.empty() && mPrefs.removeCamMotion)
        mTrafos = ct::matches::getTrafos(mMatchFolder, GeometricType::Homography);

    if (!unariesComputed())
        throw ModelException("Unaries need to be computed before optimization.");

    Detections detections;
    spdlog::info("{}", mTrackerSettings);
    if (chunk == -1)
        detections = Tracker::track(mUnaries, mManualUnaries, mTrackerSettings, mTrafos);
    else
        detections = Tracker::track(mUnaries, mManualUnaries, mTrackerSettings, chunk, mTrafos);

    auto& dd = mDetections.data();
    for (auto&& d : detections.data())
    {
        if (chunk == -1)
            dd.insert_or_assign(d.first, std::move(d.second));
        else
            dd[d.first] = std::move(d.second);
    }
}

void Model::runFullPipeline()
{
    extractFeatures();
    extractTrafos();
    extractUnaries();
    optimizeUnaries();
}

void Model::exportDetections(const std::filesystem::path& csvFile) const
{
    std::vector<std::size_t> ids;
    std::vector<cv::Point> pts;
    for (const auto& elem : mDetections.cdata())
    {
        ids.push_back(elem.first);
        pts.push_back(elem.second.position);
    }

    std::ofstream stream(csvFile.string(), std::ios::out);
    io::checkStream(stream, csvFile);
    for (std::size_t i = 0; i < ids.size(); i++)
        stream << pts[i].x << "," << pts[i].y << "," << ids[i] << "\n";
}
} // namespace gui
