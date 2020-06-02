#include "image-processing/util.h"

namespace ht
{
float scaledGauss2DPDF(
    float meanX, float meanY, float sigmaX, float sigmaY, float scale, float x, float y)
{
    // calc (x-x0)^2
    float z1 = std::pow(x - meanX, 2);
    // calc 2*sigmaX^2
    float n1 = static_cast<float>(2) * std::pow(sigmaX, 2);
    // calc z1/n1
    float s1 = z1 / n1;

    // calc (y-y0)^2
    float z2 = std::pow(y - meanY, 2);
    // calc 2*sigmaY^2
    float n2 = static_cast<float>(2) * std::pow(sigmaY, 2);
    // calc z2/n2
    float s2 = z2 / n2;

    // calc A*exp(-(s1+s2))
    float ret = scale * std::exp(-(s1 + s2));
    return ret;
}

cv::Mat scaledGauss2D(
    float meanX, float meanY, float sigmaX, float sigmaY, float scale, cv::Size size)
{
    cv::Mat gaussian = cv::Mat::zeros(size.height, size.width, CV_32FC1);

    for (int row = 0; row < size.height; ++row)
    {
        for (int col = 0; col < size.width; ++col)
        {
            gaussian.at<float>(row, col)
                = scaledGauss2DPDF(meanX, meanY, sigmaX, sigmaY, scale, col, row);
        }
    }
    return gaussian;
}
} // namespace ht
