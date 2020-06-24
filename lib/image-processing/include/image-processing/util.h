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

double calcAngle(const cv::Point& p, const cv::Point& p2);
cv::Point rotatePointAroundPoint(cv::Point centerPoint, double angle);

template <typename T>
T normL2(cv::Point_<T> const& pt)
{
    return std::sqrt(std::pow(pt.x, 2) + std::pow(pt.y, 2));
}

template <typename T>
T euclidianDist(const cv::Point_<T>& pt0, const cv::Point_<T>& pt1)
{
    return std::sqrt(std::pow(pt0.x - pt1.x, 2) + std::pow(pt0.y - pt1.y, 2));
}

template <typename InputIt>
auto mean(InputIt first, InputIt last) -> typename InputIt::value_type
{
    using T = typename InputIt::value_type;
    T sum = std::accumulate(first, last, static_cast<T>(0));
    return sum / static_cast<T>(std::distance(first, last));
}

template <typename InputIt>
auto std_dev(InputIt first, InputIt last) -> typename InputIt::value_type
{
    using T = typename InputIt::value_type;
    auto size = std::distance(first, last);

    T vec_mean = mean(first, last);
    std::vector<T> diff(size);
    std::transform(first, last, std::begin(diff), std::bind2nd(std::minus<T>(), vec_mean));

    T sq_sum
        = std::inner_product(std::begin(diff), std::end(diff), std::begin(diff), static_cast<T>(0));
    return std::sqrt(sq_sum / static_cast<T>(size));
}

inline double degree2Radian(double degree) { return ((degree * M_PI) / 180.0); }
inline double radian2Degree(double radian) { return ((radian * 180.0) / M_PI); }

} // namespace ht
#endif // HABITRACK_UTIL_H
