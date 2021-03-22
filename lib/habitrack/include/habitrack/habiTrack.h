#ifndef MODEL_HABITRACK_H
#define MODEL_HABITRACK_H


#include "habitrack/preferences.h"
#include "tracker/tracker.h"
#include "util/threadPool.h"
#include "tracker/detections.h"
#include "tracker/manualUnaries.h"
#include "tracker/unaries.h"
#include "image-processing/features.h"
#include "image-processing/images.h"
#include "image-processing/matches.h"
#include "progressbar/baseProgressBar.h"
#include <deque>
#include <filesystem>
#include <unordered_set>

namespace ht
{
class HabiTrackException : public std::exception
{
public:
    HabiTrackException(const std::string& type = std::string())
        : mType(std::string("HabiTrack: ") + type)
    {
    }
    const char* what() const throw() { return mType.c_str(); }

private:
    std::string mType;
};

class HabiTrack
{
public:
    HabiTrack();

    void setProgressBar(std::shared_ptr<BaseProgressBar> bar) { mBar = std::move(bar); }

    bool featureComputed() const;
    bool matchesComputed() const;
    bool unariesComputed() const;
    bool detectionsComputed() const;

    bool featureLoaded() const;
    bool unariesLoaded() const;
    bool detectionsLoaded() const;
    /* bool manualUnariesLoaded() const; */

    void loadImageFolder(const std::filesystem::path& imgFolder);
    void loadResultsFile(const std::filesystem::path& resultFile);
    void saveResultsFile();

    void setPreferences(const Preferences& prefs);
    Preferences getPreferences() const;

    void setStartFrame(std::size_t frame);
    void setEndFrame(std::size_t frame);
    std::size_t getStartFrame() const { return mStartFrameNumber; }
    std::size_t getEndFrame() const { return mEndFrameNumber; }

    void extractFeatures();
    void extractTrafos();
    void extractUnaries();
    std::vector<double> getUnaryQualities();
    void optimizeUnaries(int chunkId = -1);

    bool hasUsableTrafos() const;

    void runFullPipeline();

    void addManualUnary(std::size_t frame, cv::Point pt) { mManualUnaries.insert(frame, pt); }
    void removeManualUnary(std::size_t frame) { mManualUnaries.clear(frame); }

    const Images& images() const { return mImages; }
    const Features& features() const { return mFeatures; }
    const Unaries& unaries() const { return mUnaries; }
    const ManualUnaries& manualUnaries() const { return mManualUnaries; }
    const Detections& detections() const { return mDetections; }
    PairwiseMatches matches() const;
    PairwiseTrafos trafos() const;

private:
    void populatePaths();
    void openImagesHelper(const std::filesystem::path& path = {});
    std::string getDateTimeString() const;

    void setTrackerSettings(const Preferences& prefs);

private:
    std::shared_ptr<BaseProgressBar> mBar;

    Preferences mPrefs;
    Tracker::Settings mTrackerSettings;

    std::filesystem::path mOutputPath;
    std::filesystem::path mResultsFile;
    std::filesystem::path mImgFolder;
    std::filesystem::path mFtFolder;
    std::filesystem::path mMatchFolder;
    std::filesystem::path mUnFolder;
    std::filesystem::path mDetectionsFile;

    std::size_t mStartFrameNumber;
    std::size_t mEndFrameNumber;

    Images mImages;
    Features mFeatures;
    Unaries mUnaries;
    ManualUnaries mManualUnaries;
    Detections mDetections;
    std::vector<double> mUnaryQualities;

    PairwiseTrafos mTrafos;
};
} // namespace ht

#endif // MODEL_HABITRACK_H
