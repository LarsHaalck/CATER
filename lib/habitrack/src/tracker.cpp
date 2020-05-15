#include "habitrack/tracker.h"

#include "habitrack/unaries.h"
#include <iostream>
#include <opencv2/imgproc.hpp>

constexpr int perThread = 100;

namespace ht
{
void Tracker::track(const Unaries& unaries, UnarySettings unarySettings,
    SmoothBearingSettings smoothBearingSettings)
{
    auto kernel = get2DGaussianKernel(unarySettings.pairwiseSize, unarySettings.pairwiseSigma);
    auto ids = unaries.getIDs();
    auto numUnaries = ids.size();

    std::cout << numUnaries << std::endl;
}
cv::Mat Tracker::get2DGaussianKernel(int size, double sigma)
{
    cv::Mat kernelX = cv::getGaussianKernel(size, sigma, CV_32F);
    cv::Mat kernelY = cv::getGaussianKernel(size, sigma, CV_32F);
    return kernelX * kernelY.t();
}
} // namespace ht
