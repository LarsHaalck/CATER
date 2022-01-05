#ifndef HABITRACK_GPSINTERPOLATOR_H
#define HABITRACK_GPSINTERPOLATOR_H

#include "panorama/settings.h"

namespace ht
{
using GPSMap = std::unordered_map<std::size_t, cv::Point2d>;
class GPSInterpolator
{
public:
    GPSInterpolator() = default;
    GPSInterpolator(const GPSSettings& settings, std::size_t start_frame);

    bool has_prior() const { return mSettings.has_prior(); }
    GPSMap interpolate(const std::vector<std::size_t>& keyFrames) const;

private:
    GPSSettings mSettings;
    std::size_t mStartFrame;
};
} // ht

#endif // HABITRACK_GPSINTERPOLATOR_H
