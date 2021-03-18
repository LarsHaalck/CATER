#ifndef GUI_IMAGE_VIEWER_H
#define GUI_IMAGE_VIEWER_H

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <opencv2/core.hpp>

namespace ht
{
class Images;
class Unaries;
class ManualUnaries;
class Detections;
}

namespace gui
{
class ImageViewer
{
private:
    struct CacheItem
    {
        cv::Mat img;
        cv::Mat unary;
    };

public:
    ImageViewer(const ht::Images& images, const ht::Unaries& unaries,
        const ht::ManualUnaries& manualUnaries, const ht::Detections& detections);

    cv::Mat getFrame(int frameNum);

private:
    void threadEntrypoint();
    void removeStale(std::pair<std::size_t, std::size_t> range);
    void rebuildCache();
    bool insertOnMiss(int frameNum);

    CacheItem readItem(int frameNum) const;
    cv::Mat processItem(const CacheItem& item) const;
    std::pair<std::size_t, std::size_t> getRange(int frameNum) const;

private:
    const ht::Images& mImages;
    const ht::Unaries& mUnaries;
    const ht::ManualUnaries& mManualUnaries;
    const ht::Detections& mDetections;

    bool mCachedUnaries;

    std::atomic<int> mCurrent;
    std::unordered_map<std::size_t, CacheItem> mCache;
    std::mutex mMutex;
    std::condition_variable mCondition;

    std::thread mThread;
};

} // namespace gui
#endif // GUI_IMAGE_VIEWER_H
