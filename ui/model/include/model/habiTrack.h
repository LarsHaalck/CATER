#ifndef MODEL_HABITRACK_H
#define MODEL_HABITRACK_H


#include "model/preferences.h"
#include "habitrack/detections.h"
#include "habitrack/manualUnaries.h"
#include "habitrack/unaries.h"
#include "image-processing/features.h"
#include "image-processing/images.h"
#include "image-processing/matches.h"
#include "progressbar/baseProgressBar.h"
#include <deque>
#include <filesystem>
#include <unordered_set>

namespace model
{
class HabiTrack
{
public:
    bool featureComputed() const;
    bool matchesComputed() const;
    bool unariesComputed() const;
    bool detectionsComputed() const;

    void loadImageFolder(const std::filesystem::path& imgFolder);
    void loadResultsFile(const std::filesystem::path& resultFile);
    void saveResults(const std::filesystem::path& resultFile);

    void setPreferences(const model::Preferences& prefs);

    void setStartFrame(std::size_t frame);
    void setEndFrame(std::size_t frame);

    void extractFeatures();
    void extractTrafos();
    void extractUnaries();
    void optimizeUnaries(int chunkId = -1);
    void runFullPipeline();

private:
    void populatePaths();
    void openImagesHelper(const std::filesystem::path& path = {});

/* private slots: */


/*     void onPositionChanged(QPointF position); */
/*     void onBearingChanged(QPointF position); */
/*     void onPositionCleared(); */
/*     void onBearingCleared(); */

/*     void onDetectionsAvailable(int chunkId); */

private:
    std::shared_ptr<ht::BaseProgressBar> mBar;

    model::Preferences mPrefs;

    std::filesystem::path mOutputPath;
    std::filesystem::path mResultsFile;
    std::filesystem::path mImgFolder;
    std::filesystem::path mFtFolder;
    std::filesystem::path mMatchFolder;
    std::filesystem::path mUnFolder;
    std::filesystem::path mDetectionsFile;
    std::filesystem::path mSetFile;

    ht::Images mImages;
    std::size_t mCurrentFrameNumber;
    std::size_t mStartFrameNumber;
    std::size_t mEndFrameNumber;

    ht::Features mFeatures;
    ht::Unaries mUnaries;
    ht::ManualUnaries mManualUnaries;
    std::vector<double> mUnaryQualities;
    std::unordered_set<std::size_t> mInvisibles;

    ht::Detections mDetections;
    std::unordered_map<int, std::unique_ptr<QFutureWatcher<ht::Detections>>> mDetectionsWatchers;
    std::deque<int> mDetectionsQueue;
};
} // namespace model

#endif // MODEL_HABITRACK_H
