#ifndef CATER_GPSINTERPOLATOR_H
#define CATER_GPSINTERPOLATOR_H

#include <cater/panorama/settings.h>

namespace ct
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

#endif // CATER_GPSINTERPOLATOR_H
