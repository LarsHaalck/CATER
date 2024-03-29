#ifndef MODEL_CATER_H
#define MODEL_CATER_H

#include <cater/model/preferences.h>
#include <cater/image-processing/features.h>
#include <cater/image-processing/images.h>
#include <cater/image-processing/matches.h>
#include <cater/progressbar/baseProgressBar.h>
#include <cater/tracker/detections.h>
#include <cater/tracker/manualUnaries.h>
#include <cater/tracker/tracker.h>
#include <cater/tracker/unaries.h>
#include <cater/util/threadPool.h>
#include <deque>
#include <filesystem>
#include <optional>
#include <unordered_set>

namespace ct
{
class ModelException : public std::exception
{
public:
    ModelException(const std::string& type = std::string())
        : mType(std::string("Model: ") + type)
    {
    }
    const char* what() const throw() { return mType.c_str(); }

private:
    std::string mType;
};

class Model
{
public:
    Model();

    void setProgressBar(std::shared_ptr<BaseProgressBar> bar) { mBar = std::move(bar); }

    bool featureComputed() const;
    bool matchesComputed() const;
    bool unariesComputed() const;
    bool detectionsComputed() const;

    bool featureLoaded() const;
    bool unariesLoaded() const;
    bool detectionsLoaded() const;

    void unload(bool deleteMatches = false);

    void loadImageFolder(const std::filesystem::path& imgFolder);
    void loadResultsFile(const std::filesystem::path& resultFile);
    void saveResultsFile();
    void saveDetections(std::optional<std::filesystem::path> detectionsFile = {});
    void saveManualUnaries();

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
    void addManualBearing(std::size_t frame, cv::Point pt) { mDetections.insertBearing(frame, pt); }
    void removeManualBearing(std::size_t frame) { mDetections.clearManualBearing(frame); }

    void generateVideo(const std::filesystem::path& videofile) const;

    const Images& images() const { return mImages; }
    const Features& features() const { return mFeatures; }
    const Unaries& unaries() const { return mUnaries; }
    const ManualUnaries& manualUnaries() const { return mManualUnaries; }
    const Detections& detections() const { return mDetections; }
    PairwiseMatches matches() const;
    PairwiseTrafos trafos() const;

    std::filesystem::path getOutputPath() const { return mOutputPath; }

    void exportDetections(const std::filesystem::path& csvFile) const;

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

    // these are INCLUSIVE frame numbers
    std::size_t mStartFrameNumber;
    std::size_t mEndFrameNumber;

    Images mImages;
    Features mFeatures;
    Unaries mUnaries;
    ManualUnaries mManualUnaries;
    Detections mDetections;
    PairwiseTrafos mTrafos;
};
} // namespace ct

#endif // MODEL_CATER_H
