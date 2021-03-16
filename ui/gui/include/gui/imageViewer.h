#ifndef GUI_IMAGE_VIEWER_H
#define GUI_IMAGE_VIEWER_H

#include <deque>
#include <future>
#include <thread>

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
        /* cv::Mat unary; */
    };

public:
    ImageViewer(const ht::Images& images, const ht::Unaries& unaries,
        const ht::ManualUnaries& manualUnaries, const ht::Detections& detections);

    cv::Mat showFrame(int frameNum);

private:
    void buildCache();
    void rebalance();

    bool isHit(int frameNum);
    bool isSafe(int frameNum);

    CacheItem readItem(int frameNum);

    cv::Mat processItem(const CacheItem& item);

private:
    const ht::Images& mImages;
    const ht::Unaries& mUnaries;
    const ht::ManualUnaries& mManualUnaries;
    const ht::Detections& mDetections;

    int mStart;
    std::deque<CacheItem> mCache;

    std::thread mThread;
};

} // namespace gui
#endif // GUI_IMAGE_VIEWER_H
