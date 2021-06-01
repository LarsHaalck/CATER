#ifndef HT_IMAGE_VIEWER_H
#define HT_IMAGE_VIEWER_H

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <opencv2/core.hpp>
#include "image-processing/matchesTypes.h"

namespace ht
{
class HabiTrack;
}

namespace ht
{
class ImageViewer
{

public:
    struct VisSettings
    {
        int unary = 0;
        bool showManual = true;
        bool detection = true;
        bool bearing = true;
        bool trajectory = false;
        std::size_t trajectoryLength = 0;
        int radius = 20;
    };

private:
    struct CacheItem
    {
        cv::Mat img;
        cv::Mat unary;
    };

public:
    ImageViewer(const ht::HabiTrack& habiTrack, bool disableCaching = false);
    ~ImageViewer();

    cv::Mat getFrame(int frameNum, const VisSettings& settings);
    void reset();

private:
    void removeStale(std::pair<std::size_t, std::size_t> range);
    void rebuildCache();
    bool insertOnMiss(int frameNum);

    CacheItem readItem(int frameNum) const;
    cv::Mat processItem(int frameNum, const CacheItem& item, const VisSettings& settings) const;
    std::pair<std::size_t, std::size_t> getRange(int frameNum) const;

private:
    const ht::HabiTrack& mHabiTrack;
    PairwiseTrafos mTrafos;

    bool mCachedUnaries;

    std::atomic<int> mCurrent;
    std::unordered_map<std::size_t, CacheItem> mCache;
    std::mutex mMutex;
    std::condition_variable mCondition;

    std::atomic<bool> mQuit;
    std::thread mThread;
};

} // namespace ht
#endif // HT_IMAGE_VIEWER_H
