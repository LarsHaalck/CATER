#ifndef HABITRACK_DETECTIONS_H
#define HABITRACK_DETECTIONS_H

#include <opencv2/core.hpp>
#include <unordered_map>
#include "image-processing/matches.h"

namespace ht
{
struct Detection
{
    cv::Point position;
    double theta;
    double thetaQuality;
};

class Detections
{
public:
    Detections() = default;
    void insert(std::size_t idx, const Detection& detection);
    Detection at(std::size_t idx) const;
    bool exists(std::size_t idx) const;
    std::size_t size() const;

    std::vector<cv::Point2d> projectTo(std::size_t from, std::size_t to,
        const matches::PairwiseTrafos& trafos, GeometricType type) const;
    std::vector<cv::Point2d> projectFrom(std::size_t from, std::size_t to,
        const matches::PairwiseTrafos& trafos, GeometricType type) const;

    std::unordered_map<std::size_t, Detection>& data() { return mDetections; }

private:
    std::unordered_map<std::size_t, Detection> mDetections;
};
} // namespace ht
#endif // HABITRACK_DETECTIONS_H
