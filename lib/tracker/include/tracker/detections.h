#ifndef TRACKER_DETECTIONS_H
#define TRACKER_DETECTIONS_H

#include "image-processing/matches.h"
#include <map>
#include <opencv2/core.hpp>

// TODO: replace member with typedef to make data() function oboslete
namespace ht
{
struct Detection
{
    cv::Point position;
    // position quality might be added here
    // double positionQuality;
    double theta;
    double thetaQuality;
};

class Detections
{
public:
    Detections() = default;
    void insert(std::size_t idx, const Detection& detection);
    void insertBearing(std::size_t idx, const cv::Point& bearingPoint);
    void insertBearing(std::size_t idx, double theta);

    Detection at(std::size_t idx) const;
    double manualBearingAt(std::size_t idx) const;

    bool exists(std::size_t idx) const;
    bool manualBearingExists(std::size_t idx) const;
    void clearManualBearing(std::size_t idx) { mManualBearings.erase(idx); }

    std::size_t size() const;

    std::vector<cv::Point2d> projectTo(
        std::size_t from, std::size_t to, const PairwiseTrafos& trafos, GeometricType type) const;
    std::vector<cv::Point2d> projectFrom(
        std::size_t from, std::size_t to, const PairwiseTrafos& trafos, GeometricType type) const;

    std::map<std::size_t, Detection>& data() { return mDetections; }
    const std::map<std::size_t, Detection>& cdata() const { return mDetections; }

    void save(const std::filesystem::path& detectionsFile);
    static Detections fromDir(const std::filesystem::path& detectionsFile);

private:
    std::map<std::size_t, Detection> mDetections;
    std::unordered_map<std::size_t, double> mManualBearings;
};
} // namespace ht
#endif // TRACKER_DETECTIONS_H
