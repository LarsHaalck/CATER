#include "habitrack/habiTrack.h"

#include "habitrack/imageViewer.h"
#include "habitrack/resultsIO.h"
#include "image-processing/util.h"
#include "io/io.h"
#include "progressbar/progressBar.h"
#include "tracker/tracker.h"
#include "util/log.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

namespace ht
{
HabiTrack::HabiTrack()
    : mBar(std::make_shared<ProgressBar>())
{
    setTrackerSettings(mPrefs);
}

bool HabiTrack::featureComputed() const
{
    return Features::isComputed(
        mImages, mFtFolder, mPrefs.featureType, mStartFrameNumber, mEndFrameNumber + 1);
}

bool HabiTrack::matchesComputed() const
{
    if (mMatchFolder.empty())
        return false;
    return matches::isComputed(mMatchFolder, GeometricType::Homography);
}

bool HabiTrack::unariesComputed() const
{
    return Unaries::isComputed(mImgFolder, mUnFolder, mStartFrameNumber, mEndFrameNumber + 1);
}

bool HabiTrack::detectionsComputed() const { return fs::is_regular_file(mDetectionsFile); }
bool HabiTrack::featureLoaded() const { return mFeatures.size(); }
bool HabiTrack::unariesLoaded() const { return mUnaries.size(); }
bool HabiTrack::detectionsLoaded() const { return mDetections.size(); }

void HabiTrack::unload(bool deleteMatches)
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

void HabiTrack::loadImageFolder(const fs::path& imgFolder)
{
    mImgFolder = fs::absolute(imgFolder);
    mImages = Images(mImgFolder);
    if (!mImages.size())
        return;

    mStartFrameNumber = 0;
    mEndFrameNumber = mImages.size() - 1;
    openImagesHelper();
}

void HabiTrack::loadResultsFile(const fs::path& resultFile)
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

    if (featureComputed())
    {
        extractFeatures();
        if (matchesComputed())
        {
            extractTrafos();
            if (unariesComputed())
            {
                extractUnaries();
                if (detectionsComputed())
                {
                    mDetections = Detections::fromDir(mDetectionsFile);
                }
            }
        }
    }

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

void HabiTrack::generateVideo(const fs::path& videoFilename) const
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
    ImageViewer viewer(*this, true);

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

void HabiTrack::saveResultsFile()
{
    saveResults(mResultsFile, mPrefs, mImgFolder, mStartFrameNumber, mEndFrameNumber);

    if (mManualUnaries.size())
        mManualUnaries.save(mUnFolder);

    if (mDetections.size())
        mDetections.save(mDetectionsFile);
}

void HabiTrack::setPreferences(const Preferences& prefs)
{
    mPrefs = prefs;
    setTrackerSettings(prefs);
}
Preferences HabiTrack::getPreferences() const { return mPrefs; }

void HabiTrack::populatePaths()
{
    spdlog::debug("HabiTrack: Generated output path: {}", mOutputPath.string());

    mResultsFile = mOutputPath / "results.yml";
    mFtFolder = mOutputPath / "fts";
    mMatchFolder = mOutputPath / "matches";
    mUnFolder = mOutputPath / "unaries";
    mDetectionsFile = mOutputPath / "detections.yml";

    setLogFileTo(mOutputPath / "logs.txt");
}

void HabiTrack::openImagesHelper(const fs::path& path)
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

std::string HabiTrack::getDateTimeString() const
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

void HabiTrack::setTrackerSettings(const Preferences& prefs)
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

void HabiTrack::setStartFrame(std::size_t frame) { mStartFrameNumber = frame; }
void HabiTrack::setEndFrame(std::size_t frame) { mEndFrameNumber = frame; }

void HabiTrack::extractFeatures()
{
    spdlog::debug("HabiTrack: Clicked Extract Features");

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

void HabiTrack::extractTrafos()
{
    spdlog::debug("HabiTrack: Clicked Extract Trafos");

    if (!featureComputed())
        throw HabiTrackException("Features need to be computed before trafos.");

    if (matchesComputed())
        mBar->done();
    else
    {
        matches::compute(mMatchFolder, GeometricType::Homography, mFeatures,
            matches::MatchType::Windowed, 2, 0.0, nullptr, mPrefs.cacheSize,
            util::getContinuousIds(mStartFrameNumber, mEndFrameNumber + 1), mBar);
    }
}

void HabiTrack::extractUnaries()
{
    spdlog::debug("HabiTrack: Extract Unaries");

    if (!matchesComputed())
        throw HabiTrackException("Trafos need to be computed before Unaries.");

    auto start = mStartFrameNumber;
    auto end = mEndFrameNumber + 1;
    if (unariesComputed())
    {
        mUnaries = Unaries::fromDir(mImgFolder, mUnFolder, start, end);
        mBar->done();
    }
    else
    {
        auto trafos = mPrefs.removeCamMotion
            ? matches::getTrafos(mMatchFolder, GeometricType::Homography)
            : PairwiseTrafos();

        mUnaries = Unaries::compute(mImgFolder, mUnFolder, start, end, mPrefs.removeRedLasers,
            mPrefs.unarySubsample, mPrefs.unarySigma, trafos, mPrefs.cacheSize, mBar);
    }

    // zero init if existent
    mManualUnaries = ManualUnaries::fromDir(
        mUnFolder, mPrefs.unarySubsample, mPrefs.manualUnarySize, mImages.getImgSize());
}

std::vector<double> HabiTrack::getUnaryQualities()
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

bool HabiTrack::hasUsableTrafos() const
{
    auto types = ht::matches::getConnectedTypes(mMatchFolder, ht::GeometricType::Homography,
        util::getContinuousIds(mStartFrameNumber, mEndFrameNumber + 1));

    if (static_cast<unsigned int>(types & ht::GeometricType::Homography))
    {
        spdlog::info("HabiTrack: Transformations usable for unary extraction.");
        return true;
    }
    else
    {
        spdlog::warn("HabiTrack: Exracted Transformations not a continous chain, \
            Consider increasing feature points.");
        return false;
    }
}

PairwiseMatches HabiTrack::matches() const
{
    return matches::getMatches(mMatchFolder, GeometricType::Homography);
}

PairwiseTrafos HabiTrack::trafos() const
{
    return matches::getTrafos(mMatchFolder, GeometricType::Homography);
}

void HabiTrack::optimizeUnaries(int chunk)
{
    spdlog::debug("HabiTrack: Optimize Unaries (chunk {})", chunk);

    if (mTrafos.empty())
        mTrafos = ht::matches::getTrafos(mMatchFolder, GeometricType::Homography);

    if (!unariesComputed())
        throw HabiTrackException("Unaries need to be computed before optimization.");

    Detections detections;
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

void HabiTrack::runFullPipeline()
{
    extractFeatures();
    extractTrafos();
    extractUnaries();
    optimizeUnaries();
}

void HabiTrack::exportDetections(const std::filesystem::path& csvFile, bool smooth) const
{
    std::vector<std::size_t> ids;
    std::vector<cv::Point> pts;
    for (const auto& elem : mDetections.cdata())
    {
        ids.push_back(elem.first);
        pts.push_back(elem.second.position);
    }

    if (smooth)
        pts = util::smoothBoundaries(pts, mPrefs.chunkSize);

    std::ofstream stream(csvFile.string(), std::ios::out);
    io::checkStream(stream, csvFile);
    for (std::size_t i = 0; i < ids.size(); i++)
        stream << pts[i].x << "," << pts[i].y << "," << ids[i] << "\n";
}
} // namespace gui
