#ifndef HABITRACK_UTIL_H
#define HABITRACK_UTIL_H

#include <cmath>
#include <filesystem>
#include <numeric>
#include <opencv2/core.hpp>
#include <vector>

namespace ht
{
inline std::vector<std::size_t> getContinuousIds(std::size_t start, std::size_t end)
{
    std::vector<std::size_t> ids(end - start);
    std::iota(std::begin(ids), std::end(ids), 0);
    return ids;
}

float scaledGauss2DPDF(
    float meanX, float meanY, float sigmaX, float sigmaY, float scale, float x, float y);

cv::Mat scaledGauss2D(
    float meanX, float meanY, float sigmaX, float sigmaY, float scale, cv::Size size);
} // namespace ht
#endif // HABITRACK_UTIL_H
