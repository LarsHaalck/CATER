#include "tracker/detections.h"

#include "image-processing/transformation.h"
#include "image-processing/util.h"
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace ht
{
using namespace transformation;

void Detections::save(const std::filesystem::path& detectionsFile)
{
    cv::FileStorage fs(detectionsFile.string(), cv::FileStorage::WRITE);
    spdlog::info("Writing detections file to {}", detectionsFile.string());

    fs << "detections"
       << "[";

    for (const auto& elem : mDetections)
    {
        auto d = elem.second;
        fs << "{";
        fs << "frame_no" << static_cast<int>(elem.first);
        fs << "position" << d.position;
        fs << "theta" << d.theta;
        fs << "theta_quality" << d.thetaQuality;

        if (mManualBearings.count(elem.first))
        {
            fs << "manually_set" << true;
            fs << "manual_theta" << mManualBearings[elem.first];
        }
        else
        {
            fs << "manually_set" << false;
        }
        fs << "}";
    }
    fs << "]";
    fs.release();
}

Detections Detections::fromDir(const std::filesystem::path& detectionsFile)
{
    cv::FileStorage fs(detectionsFile.string(), cv::FileStorage::READ);
    spdlog::info("Reading detections file from {}", detectionsFile.string());

    Detections detections;
    cv::FileNode node = fs["detections"];
    for (cv::FileNodeIterator it = node.begin(); it != node.end(); ++it)
    {
        int frameNo = (*it)["frame_no"];
        cv::Point position;
        (*it)["position"] >> position;
        double theta;
        (*it)["theta"] >> theta;
        double thetaQuality;
        (*it)["theta_quality"] >> thetaQuality;

        int iManuallySet = (*it)["manually_set"];
        bool manuallySet = (iManuallySet == 1) ? true : false;

        if (manuallySet)
        {
            double manualTheta;
            (*it)["manual_theta"] >> manualTheta;
            detections.insertBearing(frameNo, manualTheta);
        }

        detections.insert(frameNo, {position, theta, thetaQuality});
    }
    return detections;
}

void Detections::insert(std::size_t idx, const Detection& detection)
{
    mDetections.insert({idx, detection});
}

void Detections::insertBearing(std::size_t idx, const cv::Point& bearingPoint)
{
    mManualBearings.insert({idx, util::calcAngle(mDetections[idx].position, bearingPoint)});
}

void Detections::insertBearing(std::size_t idx, double theta)
{
    mManualBearings.insert({idx, theta});
}

Detection Detections::at(std::size_t idx) const { return mDetections.at(idx); }
double Detections::manualBearingAt(std::size_t idx) const { return mManualBearings.at(idx); }

bool Detections::exists(std::size_t idx) const { return mDetections.count(idx); }
bool Detections::manualBearingExists(std::size_t idx) const { return mManualBearings.count(idx); }

std::size_t Detections::size() const { return mDetections.size(); }

std::vector<cv::Point2d> Detections::projectTo(
    std::size_t from, std::size_t to, const PairwiseTrafos& trafos, GeometricType type) const
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
        cv::Mat currTrafo;
        if (trafos.count({i, i + 1}))
            currTrafo = makeFull(trafos.at({i, i + 1}));
        else
            currTrafo = getIdentity(true);

        for (std::size_t k = from; k <= i; k++)
            points2d[k - from] = transformPoint(points2d[k - from], currTrafo, type);
    }

    return points2d;
}

std::vector<cv::Point2d> Detections::projectFrom(
    std::size_t from, std::size_t to, const PairwiseTrafos& trafos, GeometricType type) const
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
        cv::Mat currTrafo;
        if (trafos.count({i, i + 1}))
            currTrafo = invert(trafos.at({i, i + 1}), type, true);
        else
            currTrafo = getIdentity(true);

        for (std::size_t k = i; k < to; k++)
            points2d[k - from] = transformPoint(points2d[k - from], currTrafo, type);
    }

    return points2d;
}

} // namespace ht
