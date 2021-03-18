#include "gui/imageViewer.h"

#include "image-processing/images.h"
#include "tracker/detections.h"
#include "tracker/manualUnaries.h"
#include "tracker/unaries.h"

#include <spdlog/spdlog.h>

/* constexpr int left = 10; */
/* constexpr int right = 8 * left; */
/* constexpr int n = left + right; */
constexpr int left = 10;
constexpr int k = 8;
constexpr int right = k*left;

/* constexpr int leftHard = 2; */
/* constexpr int rightHard = 8; */
/* constexpr int leftSoft = 1; */
/* constexpr int rightSoft = 2; */
/* constexpr int n = leftHard + rightHard; */

namespace gui
{
ImageViewer::ImageViewer(const ht::Images& images, const ht::Unaries& unaries,
    const ht::ManualUnaries& manualUnaries, const ht::Detections& detections)
    : mImages(images)
    , mUnaries(unaries)
    , mManualUnaries(manualUnaries)
    , mDetections(detections)
    , mCachedUnaries(false)
    , mCurrent(-1)
    , mCache()
{
    mThread = std::thread(&ImageViewer::threadEntrypoint, this);
}

cv::Mat ImageViewer::getFrame(int frameNum)
{
    mCurrent = frameNum;

    cv::Mat frame;
    // check for hit and return it
    {
        std::scoped_lock<std::mutex> lock(mMutex);

        if (mCache.count(frameNum))
            frame = processItem(mCache[frameNum]);
    }
    mCondition.notify_all();

    // on miss, direct read
    if (frame.empty())
        frame = processItem(readItem(frameNum));

    return frame;
}

void ImageViewer::threadEntrypoint()
{
    std::unique_lock<std::mutex> lock(mMutex);
    mCondition.wait(lock);
    lock.unlock();

    rebuildCache();
}

void ImageViewer::removeStale(std::pair<std::size_t, std::size_t> range)
{
    std::scoped_lock<std::mutex> lock(mMutex);
    for (auto it = std::begin(mCache); it != std::end(mCache);)
    {
        if (it->first < range.first ||  it->first > range.second)
            it = mCache.erase(it);
        else
            ++it;
    }
}

void ImageViewer::rebuildCache()
{
    // rebuild cache for mCurrent
    while (true)
    {
        int item = mCurrent.load();
        auto range = getRange(item);

        // cleanup everything else
        removeStale(range);

        bool finished = true;
        for (std::size_t i = item; i < range.second; i++)
        {
            if(insertOnMiss(i))
            {
                finished = false;
                break;
            }
        }

        for (std::size_t i = range.first; i < static_cast<std::size_t>(item); i++)
        {
            if(insertOnMiss(i))
            {
                finished = false;
                break;
            }
        }

        if (finished)
        {
            std::unique_lock<std::mutex> lock(mMutex);
            mCondition.wait(lock);
        }
    }
}

bool ImageViewer::insertOnMiss(int frameNum)
{
    std::unique_lock<std::mutex> lock(mMutex);
    bool hit = mCache.count(frameNum);
    lock.unlock();

    if (!hit)
    {
        auto item = readItem(frameNum);

        lock.lock();
        mCache.insert({frameNum, item});
        lock.unlock();

        return true;
    }

    return false;
}

std::pair<std::size_t, std::size_t> ImageViewer::getRange(int frameNum) const
{
    auto start = std::max<int>(0, frameNum - left);
    auto end = std::min<int>(frameNum + right, mImages.size());
    return {start, end};
}

ImageViewer::CacheItem ImageViewer::readItem(int frameNumber) const
{
    CacheItem item;
    item.img = mImages.at(frameNumber);

    if (mUnaries.exists(frameNumber))
        item.unary = mUnaries.at(frameNumber);

    return item;
}

cv::Mat ImageViewer::processItem(const CacheItem& item) const { return item.img; }
} // namespace gui
