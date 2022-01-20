#include <habitrack/tracker/manualUnaries.h>

#include <habitrack/image-processing/util.h>
#include <opencv2/imgproc.hpp>

#include "unaryIO.h"

namespace ht
{
namespace fs = std::filesystem;

ManualUnaries::ManualUnaries()
    : mSubsample(0)
    , mUnarySize(0)
    , mImgSize()
    , mPoints()
    , mBluePrint()
{
}

ManualUnaries::ManualUnaries(double subsample, int unarySize, cv::Size imgSize)
    : mSubsample(subsample)
    , mUnarySize(unarySize)
    , mImgSize(imgSize)
    , mPoints()
    , mBluePrint(getBluePrint())
{
}

ManualUnaries::ManualUnaries(double subsample, int unarySize, cv::Size imgSize,
    const std::unordered_map<std::size_t, cv::Point2f>& points)
    : mSubsample(subsample)
    , mUnarySize(unarySize)
    , mImgSize(imgSize)
    , mPoints(points)
    , mBluePrint(getBluePrint())
{
}

ManualUnaries ManualUnaries::fromDir(
    const std::filesystem::path& unDir, double subsample, int unarySize, cv::Size imgSize)
{
    auto file = unDir / "manual_unaries.json";
    if (!fs::is_regular_file(file))
        return ManualUnaries(subsample, unarySize, imgSize);
    std::ifstream stream(file.string(), std::ios::in);
    io::checkStream(stream, file);
    std::unordered_map<std::size_t, cv::Point2f> points;
    {
        cereal::JSONInputArchive archive(stream);
        archive(points);
    }

    ManualUnaries manual(subsample, unarySize, imgSize, points);
    return manual;
}

void ManualUnaries::save(const std::filesystem::path& unDir)
{
    auto file = unDir / "manual_unaries.json";
    std::ofstream stream(file.string(), std::ios::out);
    io::checkStream(stream, file);
    {
        cereal::JSONOutputArchive archive(stream);
        archive(mPoints);
    }
}

cv::Mat ManualUnaries::unaryAt(std::size_t id) const
{
    assert(mPoints.count(id) && "Unary id does not exist in unaryAt()");
    cv::Mat unary = getUnaryFromPoint(mPoints.at(id));
    unary += 0.0001f;
    return unary;
}

cv::Mat ManualUnaries::previewUnaryAt(std::size_t id) const
{
    assert(mPoints.count(id) && "Unary id does not exist in previewUnaryAt()");
    cv::Mat unary = getUnaryFromPoint(mPoints.at(id));
    unary *= 255;
    cv::resize(unary, unary, mImgSize);
    unary.convertTo(unary, CV_8UC1);
    return unary;
}

cv::Point2f ManualUnaries::unaryPointAt(std::size_t id) const
{
    assert(mPoints.count(id) && "Unary id does not exist in unaryAt()");
    return mPoints.at(id);
}

void ManualUnaries::insert(std::size_t id, cv::Point2f pt) { mPoints.insert_or_assign(id, pt); }

void ManualUnaries::clear(std::size_t id) { mPoints.erase(id); }

bool ManualUnaries::exists(std::size_t id) const { return mPoints.count(id); }

cv::Mat ManualUnaries::getUnaryFromPoint(const cv::Point2f& pt) const
{
    auto targetScaled = 1.5 * cv::Point2f(mImgSize.width, mImgSize.height) - pt;
    targetScaled = mSubsample * targetScaled;
    auto sizeScaled = cv::Size(mSubsample * mImgSize.width, mSubsample * mImgSize.height);
    auto srcRect = cv::Rect(targetScaled, sizeScaled);

    cv::Mat translated = cv::Mat::zeros(sizeScaled, mBluePrint.type());
    mBluePrint(srcRect).copyTo(translated);
    return translated;
}

cv::Mat ManualUnaries::getBluePrint()
{
    auto center = 1.5 * cv::Point2f(mImgSize.width, mImgSize.height);
    auto size = cv::Size(3 * mImgSize.width, 3 * mImgSize.height);
    cv::Mat gaussian = util::scaledGauss2D(center.x, center.y, mUnarySize, mUnarySize, 1.0, size);
    cv::resize(gaussian, gaussian, cv::Size(), mSubsample, mSubsample, cv::INTER_LINEAR);
    return gaussian;
}
} // namespace ht
