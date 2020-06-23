#include "habitrack/detections.h"

#include "image-processing/transformation.h"
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace ht
{
using namespace transformation;

void Detections::save(const std::filesystem::path& detectionsFile)
{
    cv::FileStorage fs(detectionsFile.string(),
        cv::FileStorage::WRITE);
    spdlog::info("Writing detections file to {}", detectionsFile.string());

    std::vector<std::pair<std::size_t, Detection>> elems(
        std::begin(mDetections), std::end(mDetections));

    std::sort(std::begin(elems), std::end(elems),
        [](const auto& p1, const auto& p2) { return p1.first < p2.first; });

    fs << "detections"
       << "[";

    for (const auto& elem : elems)
    {
            auto d = elem.second;
            fs << "{";
            fs << "frame_no" << static_cast<int>(elem.first);
            fs << "position" << d.position;
            fs << "theta" << d.theta;
            fs << "theta_quality" << d.thetaQuality;

            cv::Point manualThetaPosition;
            /* if (trackingData.getManuallySetBearingDirectionPoint( */
            /*         *it, manualThetaPosition)) */
            /* { */
            /*     fs << "manually_set" << true; */
            /*     fs << "manual_direction_point" << manualThetaPosition; */
            /* } */
            /* else */
            /* { */
            /*     fs << "manually_set" << false; */
            /* } */
            fs << "}";
    }
    fs << "]";
    fs.release();
}

static Detections fromDir(const std::filesystem::path& detectionsFile)
{
    return Detections();
}

void Detections::insert(std::size_t idx, const Detection& detection)
{
    mDetections.insert({idx, detection});
}

Detection Detections::at(std::size_t idx) const { return mDetections.at(idx); }

bool Detections::exists(std::size_t idx) const { return mDetections.count(idx); }
std::size_t Detections::size() const { return mDetections.size(); }

std::vector<cv::Point2d> Detections::projectTo(std::size_t from, std::size_t to,
    const matches::PairwiseTrafos& trafos, GeometricType type) const
{
    if (from >= to)
        return {};

    // find first existing index
    for (std::size_t i = from; i < to; i++)
    {
        if (mDetections.count(i))
        {
            from = i;
            break;
        }
    }

    std::vector<cv::Point2d> points2d;
    for (std::size_t i = from; i < to; i++)
        points2d.push_back(mDetections.at(i).position);

    for (std::size_t i = from; i < to; i++)
    {
        auto currTrafo = makeFull(trafos.at({i, i + 1}));
        for (std::size_t k = from; k <= i; k++)
            points2d[k - from] = transformPoint(points2d[k - from], currTrafo, type);
    }

    return points2d;
}

std::vector<cv::Point2d> Detections::projectFrom(std::size_t from, std::size_t to,
    const matches::PairwiseTrafos& trafos, GeometricType type) const
{
    if (from >= to)
        return {};

    // find last existing index
    for (std::size_t i = from; i < to; i++)
    {
        if (!mDetections.count(i))
        {
            to = i - 1;
            break;
        }
    }

    std::vector<cv::Point2d> points2d;
    for (std::size_t i = from; i < to; i++)
    {
        if (mDetections.count(i))
            points2d.push_back(mDetections.at(i).position);
    }

    for (std::size_t i = from; i < to; i++)
    {
        auto currTrafo = invert(trafos.at({i, i + 1}), type, true);
        for (std::size_t k = i; k < to; k++)
            points2d[k - from] = transformPoint(points2d[k - from], currTrafo, type);
    }

    return points2d;
}

} // namespace ht
