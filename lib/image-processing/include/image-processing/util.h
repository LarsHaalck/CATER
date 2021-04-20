#ifndef HABITRACK_UTIL_H
#define HABITRACK_UTIL_H

#include <cmath>
#include <filesystem>
#include <numeric>
#include <opencv2/core.hpp>
#include <vector>

namespace ht::util
{
inline std::vector<std::size_t> getContinuousIds(std::size_t start, std::size_t end)
{
    std::vector<std::size_t> ids(end - start);
    std::iota(std::begin(ids), std::end(ids), start);
    return ids;
}

float gauss1DPDF(float mean, float sigma, float x);
float scaledGauss2DPDF(
    float meanX, float meanY, float sigmaX, float sigmaY, float scale, float x, float y);

cv::Mat scaledGauss2D(
    float meanX, float meanY, float sigmaX, float sigmaY, float scale, cv::Size size);

double calcAngle(const cv::Point2d& p, const cv::Point2d& p2);

cv::Point rotatePointAroundPoint(cv::Point centerPoint, double angle, int radius);
void highlightImg(cv::Mat& img);

template <typename T>
double normL2(const cv::Point_<T>& pt)
{
    return std::sqrt(std::pow(pt.x, 2) + std::pow(pt.y, 2));
}

template <typename T>
T euclidianDist(const cv::Point_<T>& pt0, const cv::Point_<T>& pt1)
{
    return std::sqrt(std::pow(pt0.x - pt1.x, 2) + std::pow(pt0.y - pt1.y, 2));
}

cv::Mat overlayPoints(const cv::Mat& img, const std::vector<cv::Point2d>& pts,
    const std::vector<std::size_t>& sizes = {});

} // namespace ht
#endif // HABITRACK_UTIL_H
