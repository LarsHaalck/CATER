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
class HabiTrack;
}

namespace gui
{
class ImageViewer
{

public:
    struct VisSettings
    {
        int unary;
        bool detection;
        bool bearing;
        bool trajectory;
        std::size_t trajectoryLength;
    };

private:
    struct CacheItem
    {
        cv::Mat img;
        cv::Mat unary;
    };

public:
    ImageViewer(const ht::HabiTrack& habiTrack);
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

    bool mCachedUnaries;

    std::atomic<int> mCurrent;
    std::unordered_map<std::size_t, CacheItem> mCache;
    std::mutex mMutex;
    std::condition_variable mCondition;

    std::atomic<bool> mQuit;
    std::thread mThread;
};

} // namespace gui
#endif // GUI_IMAGE_VIEWER_H
