#include "habitrack/manualUnaries.h"

#include "image-processing/util.h"
#include <opencv2/imgproc.hpp>

#include "unaryIO.h"

namespace ht
{
namespace fs = std::filesystem;
ManualUnaries::ManualUnaries()
    : mSubsample(0)
    , mImgSize()
{
}

ManualUnaries::ManualUnaries(double subsample, cv::Size imgSize)
    : mSubsample(subsample)
    , mImgSize(imgSize)
    , mPoints()
    , mUnaries()
{
}

ManualUnaries::ManualUnaries(
    double subsample, cv::Size imgSize, const std::unordered_map<std::size_t, cv::Point2f>& points)
    : mSubsample(subsample)
    , mImgSize(imgSize)
    , mPoints(points)
    , mUnaries()
{
    mUnaries.reserve(mPoints.size());
    for (const auto& [id, pt] : mPoints)
    {
        cv::Mat gaussian = scaledGauss2D(pt.x, pt.y, 5.0, 5.0, 255.0, mImgSize);
        cv::resize(gaussian, gaussian, cv::Size(), mSubsample, mSubsample, cv::INTER_LINEAR);
        mUnaries.insert({id, gaussian});
    }
}

ManualUnaries ManualUnaries::fromDir(
    const std::filesystem::path& unDir, double subsample, cv::Size imgSize)
{
    auto file = unDir / "manual_unaries.json";
    if (!fs::is_regular_file(file))
        return ManualUnaries(subsample, imgSize);
    std::ifstream stream(file.string(), std::ios::in);
    io::checkStream(stream, file);
    std::unordered_map<std::size_t, cv::Point2f> points;
    {
        cereal::JSONInputArchive archive(stream);
        archive(points);
    }

    ManualUnaries manual(subsample, imgSize, points);
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
    assert(mUnaries.count(id) && "Unary id does not exist in unaryAt()");
    // ALWAYS clone because the mat is modified inplace otherwise
    cv::Mat unary = mUnaries.at(id).clone();
    unary.convertTo(unary, CV_32FC1);
    unary /= 255.0f;
    unary += 0.0001f;
    return unary;
}

cv::Mat ManualUnaries::previewUnaryAt(std::size_t id) const
{
    assert(mUnaries.count(id) && "Unary id does not exist in previewUnaryAt()");
    // ALWAYS clone because the mat is modified inplace otherwise
    cv::Mat unary = mUnaries.at(id).clone();
    unary.convertTo(unary, CV_8UC1);
    return unary;
}

/* cv::Point2f ManualUnaries::unaryPointAt(std::size_t id) const */
/* { */
/*     assert(mPoints.count(id) && "Unary id does not exist in unaryAt()"); */
/*     return mPoints.at(id); */
/* } */

void ManualUnaries::insert(std::size_t id, cv::Point2f pt)
{
    mPoints.insert_or_assign(id, pt);
    cv::Mat gaussian = scaledGauss2D(pt.x, pt.y, 5.0, 5.0, 255.0, mImgSize);
    cv::resize(gaussian, gaussian, cv::Size(), mSubsample, mSubsample, cv::INTER_LINEAR);
    mUnaries.insert_or_assign(id, gaussian);
}

void ManualUnaries::clear(std::size_t id)
{
    mPoints.erase(id);
    mUnaries.erase(id);
}

bool ManualUnaries::exists(std::size_t id) const { return mPoints.count(id); }
} // namespace ht
