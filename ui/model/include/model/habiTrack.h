#ifndef MODEL_HABITRACK_H
#define MODEL_HABITRACK_H


#include "model/preferences.h"
#include "tracker/tracker.h"
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
class HabiTrack
{
public:
    HabiTrack();
    bool featureComputed() const;
    bool matchesComputed() const;
    bool unariesComputed() const;
    bool detectionsComputed() const;

    void loadImageFolder(const std::filesystem::path& imgFolder);
    void loadResultsFile(const std::filesystem::path& resultFile);
    void saveResultsFile();

    void setPreferences(const Preferences& prefs);
    Preferences getPreferences() const;

    void setStartFrame(std::size_t frame);
    void setEndFrame(std::size_t frame);

    void extractFeatures();
    void extractTrafos();
    void extractUnaries();
    void optimizeUnaries(int chunkId = -1);
    void runFullPipeline();

/*     void onPositionChanged(QPointF position); */
/*     void onBearingChanged(QPointF position); */
/*     void onPositionCleared(); */
/*     void onBearingCleared(); */
/*     void onDetectionsAvailable(int chunkId); */

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
    /* std::filesystem::path mSetFile; */

    Images mImages;
    std::size_t mCurrentFrameNumber;
    std::size_t mStartFrameNumber;
    std::size_t mEndFrameNumber;

    Features mFeatures;

    Unaries mUnaries;
    ManualUnaries mManualUnaries;
    std::vector<double> mUnaryQualities;

    Detections mDetections;
};
} // namespace ht

#endif // MODEL_HABITRACK_H
