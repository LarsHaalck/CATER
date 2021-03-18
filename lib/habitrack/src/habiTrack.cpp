#include "habitrack/habiTrack.h"

#include "image-processing/util.h"
#include "resultsIO.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

namespace ht
{
HabiTrack::HabiTrack()
{
    /* mBar = std::make_shared<ProgressStatusBar>(ui->progressBar, ui->labelProgress); */
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

bool HabiTrack::detectionsComputed() const { return fs::is_regular_file(mDetectionsFile); }

void HabiTrack::loadImageFolder(const fs::path& imgFolder)
{
    mImgFolder = imgFolder;
    mImages = Images(mImgFolder);
    mStartFrameNumber = 1;
    mEndFrameNumber = mImages.size();
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
    /* if (fs::is_regular_file(mSetFile)) */
    /*     mInvisibles = loadSet(mSetFile); */

}

void HabiTrack::saveResultsFile()
{
    saveResults(mResultsFile, mPrefs, mImgFolder, mStartFrameNumber, mEndFrameNumber);
    if (mManualUnaries.size())
        mManualUnaries.save(mUnFolder);

    /* if (mInvisibles.size()) */
    /*     saveSet(mSetFile, mInvisibles); */
}

void HabiTrack::setPreferences(const Preferences& prefs)
{
    mPrefs = prefs;
    setTrackerSettings(prefs);
}
Preferences HabiTrack::getPreferences() const
{
    return mPrefs;
}

void HabiTrack::populatePaths()
{
    spdlog::debug("HabiTrack: Generated output path: {}", mOutputPath.string());

    mResultsFile = mOutputPath / "results.yml";
    mFtFolder = mOutputPath / "fts";
    mMatchFolder = mOutputPath / "matches";
    mUnFolder = mOutputPath / "unaries";
    mDetectionsFile = mOutputPath / "detections.yml";
    /* mSetFile = mOutputPath / "invisibles.yml"; */

    // TODO: uncomment
    /* try */
    /* { */
    /*     auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>(); */
    /*     console_sink->set_level(spdlog::level::info); */

    /*     auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>( */
    /*         (mOutputPath / "logs.txt").string(), false); */
    /*     file_sink->set_level(spdlog::level::debug); */

    /*     auto logger = std::make_shared<spdlog::logger>( */
    /*         "multi_sink", spdlog::sinks_init_list({console_sink, file_sink})); */
    /*     logger->set_level(spdlog::level::debug); */
    /*     spdlog::flush_every(std::chrono::seconds(3)); */
    /*     spdlog::set_default_logger(logger); */
    /*     spdlog::info("New run -------------"); */
    /* } */
    /* catch (const spdlog::spdlog_ex& ex) */
    /* { */
    /*     std::cout << "Log initialization failed: " << ex.what() << std::endl; */
    /* } */
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

void HabiTrack::extractTrafos()
{
    spdlog::debug("HabiTrack: Clicked Extract Trafos");

    if (!featureComputed())
    {
        // TODO: exception?
        spdlog::warn("HabiTrack: Features need to be computed before trafos.");
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

    auto types = ht::matches::getConnectedTypes(mMatchFolder, ht::GeometricType::Homography,
        getContinuousIds(mStartFrameNumber - 1, mEndFrameNumber));
    if (static_cast<unsigned int>(types & ht::GeometricType::Homography))
        spdlog::info("HabiTrack: Transformations usable for unary extraction.");
    else
    {
        // TODO: exception?
        spdlog::warn(
            "HabiTrack: Exracted Transformations not a continous chain, \
            Consider increasing feature points.");
    }
}

void HabiTrack::extractUnaries()
{
    spdlog::debug("HabiTrack: Clicked Extract Unaries");
    if (!matchesComputed())
    {
        // TODO: exception?
        spdlog::warn("HabiTrack: Trafos need to be computed before Unaries.");
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

    /* std::vector<double> unaryQuality(mImages.size(), -1); */
    /* for (auto i = mStartFrameNumber - 1; i < mEndFrameNumber - 1; i++) */
    /*     unaryQuality[i] = Unaries::getUnaryQuality(unaries.at(i)); */

    // zero init if existent
    mManualUnaries = ManualUnaries::fromDir(
        mUnFolder, mPrefs.unarySubsample, mPrefs.manualUnarySize, mImages.getImgSize());
}

/* void HabiTrack::optimizeUnaries(int chunk, std::function<void()> callback) */
/* { */
/*     spdlog::debug("HabiTrack: Clicked Optimize Unaries"); */
/*     if (!unariesComputed()) */
/*     { */
/*         // TODO: exception? */
/*         spdlog::warn("HabiTrack: Unaries need to be computed before pptimization."); */
/*         return; */
/*     } */

/*     Detections detections; */
/*     auto trafos = ht::matches::getTrafos(mMatchFolder, GeometricType::Homography); */
/*     if (chunk == -1) */
/*         detections = Tracker::track(mUnaries, mManualUnaries, mTrackerSettings, trafos); */
/*     else */
/*         detections = Tracker::track(mUnaries, mManualUnaries, mTrackerSettings, chunk, trafos); */

/*     auto& dd = mDetections.data(); */
/*     for (auto&& d : detections.data()) */
/*         dd.insert_or_assign(d.first, std::move(d.second)); */
/* } */

/* void HabiTrack::optimizeUnaries(int chunk, std::function<void()> callback) */
/* { */
/*     spdlog::debug("HabiTrack: Clicked Optimize Unaries"); */
/*     if (!unariesComputed()) */
/*     { */
/*         // TODO: exception? */
/*         spdlog::warn("HabiTrack: Unaries need to be computed before pptimization."); */
/*         return; */
/*     } */

/*     Detections detections; */
/*     auto trafos = ht::matches::getTrafos(mMatchFolder, GeometricType::Homography); */
/*     if (chunk == -1) */
/*         detections = Tracker::track(unaries, mManualUnaries, mTrackerSettings, trafos); */
/*     else */
/*         detections = Tracker::track(unaries, mManualUnaries, mTrackerSettings, chunk, trafos); */

/*     auto& dd = mDetections.data(); */
/*     for (auto&& d : detections.data()) */
/*         dd.insert_or_assign(d.first, std::move(d.second)); */
/* } */

/* void HabiTrack::runFullPipeline() */
/* { */
/*     extractFeatures(); */
/*     extractTrafos(); */
/*     extractUnaries(); */
/*     optimizeUnaries(); */
/* } */

/* void HabiTrack::onPositionChanged(QPointF position) */
/* { */
/*     if (!mUnaries.size()) */
/*         return; */

/*     std::size_t chunk = 0; */
/*     if (mPrefs.chunkSize) */
/*     { */
/*         chunk = (mCurrentFrameNumber - mStartFrameNumber) / mPrefs.chunkSize; */
/*         spdlog::debug("GUI: manual position changed to ({}, {}) on frame {} [chunk {}]", */
/*             position.x(), position.y(), mCurrentFrameNumber - 1, chunk); */
/*     } */
/*     else */
/*     { */
/*         spdlog::debug("GUI: manual position changed to ({}, {}) on frame {}", position.x(), */
/*             position.x(), position.y(), mCurrentFrameNumber - 1, chunk); */
/*     } */

/*     // put it in queue if it does not exist */
/*     if (std::find(std::begin(detectionsQueue), std::end(detectionsQueue), chunk) */
/*         == std::end(detectionsQueue)) */
/*     { */
/*         spdlog::debug("GUI: added chunk {} to queue", chunk); */
/*         detectionsQueue.push_back(chunk); */
/*     } */

/*     mManualUnaries.insert(mCurrentFrameNumber - 1, QtOpencvCore::qpoint2point(position)); */

/*     ui->unaryView->getUnaryScene()->setUnaryQuality( */
/*         mCurrentFrameNumber - 1, UnaryQuality::Excellent); */
/*     /1* ui->unaryView->getUnaryScene()->update(); *1/ */
/*     statusBar()->showMessage("Manually added unary", statusDelay); */
/*     on_buttonNextFrame_clicked(); */
/* } */

/* void HabiTrack::onBearingChanged(QPointF) */
/* { */
/*     /1* spdlog::debug("GUI: manual bearing changed to ({}, {}) on frame {}", position.x(), *1/ */
/*     /1*     position.y(), mCurrentFrameNumber - 1); *1/ */
/*     // TODO: implement */
/* } */

/* void HabiTrack::onPositionCleared() */
/* { */
/*     if (!mUnaries.size()) */
/*         return; */

/*     std::size_t chunk = 0; */
/*     if (mPrefs.chunkSize) */
/*     { */
/*         chunk = (mCurrentFrameNumber - mStartFrameNumber) / mPrefs.chunkSize; */
/*         spdlog::debug( */
/*             "GUI: manual position cleard on frame {} [chunk {}]", mCurrentFrameNumber - 1, chunk); */
/*     } */
/*     else */
/*         spdlog::debug("GUI: manual position cleard on frame {}", mCurrentFrameNumber - 1); */

/*     // put it in queue if it does not exist */
/*     if (std::find(std::begin(detectionsQueue), std::end(detectionsQueue), chunk) */
/*         == std::end(detectionsQueue)) */
/*     { */
/*         spdlog::debug("GUI: added chunk {} to queue", chunk); */
/*         detectionsQueue.push_back(chunk); */
/*     } */

/*     mManualUnaries.clear(mCurrentFrameNumber - 1); */
/*     showFrame(mCurrentFrameNumber); */
/*     statusBar()->showMessage("Manual unary cleared", statusDelay); */

/*     ui->unaryView->getUnaryScene()->setUnaryQuality( */
/*         mCurrentFrameNumber - 1, mUnaryQualityValues[mCurrentFrameNumber - 1]); */
/*     ui->unaryView->getUnaryScene()->update(); */
/* } */

/* void HabiTrack::onBearingCleared() */
/* { */
/*     /1* if (!mImages.size()) *1/ */
/*     /1*     return; *1/ */
/*     /1* spdlog::debug("GUI: manual bearing cleared on frame {}", mCurrentFrameNumber - 1); *1/ */
/*     // TODO: implement */
/* } */

} // namespace gui
