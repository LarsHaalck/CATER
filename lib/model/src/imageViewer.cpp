#include <habitrack/model/imageViewer.h>

#include <habitrack/model/model.h>
#include <habitrack/image-processing/util.h>

#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

constexpr int left = 10;
constexpr int k = 8;
constexpr int right = k * left;

using namespace ht;

namespace ht
{
ImageViewer::ImageViewer(const Model& habiTrack, Cache cache)
    : mModel(habiTrack)
    , mTrafos()
    , mCurrent(-1)
    , mCache()
    , mQuit(false)
{
    if (cache == Cache::Enable)
        mThread = std::thread(&ImageViewer::rebuildCache, this);
    mTrafos = mModel.trafos();
}

ImageViewer::~ImageViewer()
{
    mQuit = true;
    mCondition.notify_all();

    // not joinable when caching is disabled
    if (mThread.joinable())
        mThread.join();
}

cv::Mat ImageViewer::getFrame(int frameNum, const VisSettings& settings)
{
    mCurrent = frameNum;

    cv::Mat frame;
    // check for hit and return it
    {
        std::scoped_lock<std::mutex> lock(mMutex);

        if (mCache.count(frameNum))
            frame = processItem(frameNum, mCache[frameNum], settings);
    }
    mCondition.notify_all();

    // on miss, direct read
    if (frame.empty())
        frame = processItem(frameNum, readItem(frameNum), settings);

    return frame;
}

void ImageViewer::reset()
{
    mTrafos = mModel.trafos();
    removeStale({mModel.images().size(), 0});
    mCondition.notify_all();
}

void ImageViewer::removeStale(std::pair<std::size_t, std::size_t> range)
{
    std::scoped_lock<std::mutex> lock(mMutex);
    for (auto it = std::begin(mCache); it != std::end(mCache);)
    {
        if (it->first < range.first || it->first > range.second)
        {
            it = mCache.erase(it);
        }
        else
            ++it;
    }
}

void ImageViewer::rebuildCache()
{
    {
        std::unique_lock<std::mutex> lock(mMutex);
        mCondition.wait(lock);
    }
    if (mQuit.load())
        return;

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
            if (insertOnMiss(i))
            {
                finished = false;
                break;
            }
        }

        for (std::size_t i = range.first; i < static_cast<std::size_t>(item); i++)
        {
            if (insertOnMiss(i))
            {
                finished = false;
                break;
            }
        }

        if (mQuit.load())
            return;

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
    auto end = std::min<int>(frameNum + right, mModel.images().size());
    return {start, end};
}

ImageViewer::CacheItem ImageViewer::readItem(int frameNumber) const
{
    CacheItem item;
    item.img = mModel.images().at(frameNumber);

    if (mModel.unaries().exists(frameNumber))
        item.unary = mModel.unaries().previewAt(frameNumber);

    return item;
}

cv::Mat ImageViewer::processItem(
    int frameNum, const CacheItem& item, const VisSettings& settings) const
{
    std::size_t idx = frameNum;

    cv::Mat frame = item.img.clone(); // zero indexed here

    // overlay unary
    const auto& manualUnaries = mModel.manualUnaries();
    if (settings.unary > 0 && mModel.unaries().exists(idx))
    {
        double alpha = static_cast<double>(settings.unary) / 100.0;

        cv::Mat unary;
        if (manualUnaries.exists(idx))
            unary = manualUnaries.previewUnaryAt(idx);
        else
            unary = item.unary;

        cv::Mat unaryColor;
        cv::cvtColor(unary, unaryColor, cv::COLOR_GRAY2BGR);
        std::vector<cv::Mat> channels(3);
        cv::split(unaryColor, channels);
        channels[0] = cv::Mat::zeros(unary.size(), CV_8UC1);
        cv::merge(channels, unaryColor);
        cv::addWeighted(unaryColor, alpha, frame, (1 - alpha), 0.0, frame);
    }

    // overlay position
    const auto& detections = mModel.detections();
    if (settings.detection && detections.exists(idx))
    {
        auto detection = detections.at(idx);
        auto position = detection.position;
        if (settings.bearing)
        {
            double theta;
            cv::Scalar color;
            if (detections.manualBearingExists(idx))
            {
                theta = detections.manualBearingAt(idx);
                color = cv::Scalar(255, 255, 0);
            }
            else
            {
                theta = detection.theta;
                color = cv::Scalar(0, 255, 0);
            }
            auto dirIndicator = util::rotatePointAroundPoint(position, theta, settings.radius);
            cv::line(frame, position, dirIndicator, color, 1);
        }

        // overlay manual unary as well
        if (settings.showManual && manualUnaries.exists(idx))
        {
            auto unary = manualUnaries.previewUnaryAt(idx);
            cv::Mat unaryColor;
            cv::cvtColor(unary, unaryColor, cv::COLOR_GRAY2BGR);
            std::vector<cv::Mat> channels(3);
            cv::split(unaryColor, channels);
            channels[2] = cv::Mat::zeros(unary.size(), CV_8UC1);
            cv::merge(channels, unaryColor);
            cv::add(frame, unaryColor, frame);
        }

        cv::circle(frame, position, settings.radius, cv::Scalar(100, 100, 255), 2);
    }

    // get trajectory
    auto win = settings.trajectoryLength;
    if (settings.trajectory && win > 0 && detections.exists(idx))
    {
        std::size_t start = 0;
        if (idx > win)
            start = idx - win;

        auto track
            = detections.projectTo(start, idx, mTrafos, GeometricType::Homography);
        for (std::size_t i = 1; i < track.size(); ++i)
        {
            cv::Point pre = track.at(i - 1);
            cv::Point cur = track.at(i);
            cv::line(frame, pre, cur, cv::Scalar(0, 127, 255), 2);
        }

        track = detections.projectFrom(
            idx, idx + win, mTrafos, GeometricType::Homography);
        for (std::size_t i = 1; i < track.size(); ++i)
        {
            cv::Point pre = track.at(i - 1);
            cv::Point cur = track.at(i);
            cv::line(frame, pre, cur, cv::Scalar(0, 255, 255), 2);
        }
    }

    /* std::stringstream filename; */
    /* filename << std::setw(3) << std::setfill('0') << idx; */
    /* cv::imwrite(std::string("out/") + std::string("detections_") + filename.str() + ".png", frame); */
    return frame;
}

} // namespace ht
