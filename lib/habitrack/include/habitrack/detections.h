#ifndef HABITRACK_DETECTIONS_H
#define HABITRACK_DETECTIONS_H

#include <opencv2/core.hpp>
#include <unordered_map>

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
private:
    std::unordered_map<std::size_t, Detection> mDetections;


};
} // namespace ht
#endif // HABITRACK_DETECTIONS_H
