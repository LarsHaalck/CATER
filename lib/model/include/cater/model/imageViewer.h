#ifndef CT_IMAGE_VIEWER_H
#define CT_IMAGE_VIEWER_H

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <opencv2/core.hpp>
#include <cater/image-processing/matchesTypes.h>

namespace ct
{
class Model;
}

namespace ct
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

    enum class Cache
    {
        Enable,
        Disable
    };

private:
    struct CacheItem
    {
        cv::Mat img;
        cv::Mat unary;
    };

public:
    ImageViewer(const ct::Model& model, Cache cache);
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
    const ct::Model& mModel;
    PairwiseTrafos mTrafos;

    std::atomic<int> mCurrent;
    std::unordered_map<std::size_t, CacheItem> mCache;
    std::mutex mMutex;
    std::condition_variable mCondition;

    std::atomic<bool> mQuit;
    std::thread mThread;
};

} // namespace ct
#endif // CT_IMAGE_VIEWER_H
