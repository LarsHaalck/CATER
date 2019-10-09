#include "habitrack/grayDecorator.h"

#include <opencv2/imgproc.hpp>
#include <iostream>

namespace ht
{
GrayDecorator::GrayDecorator(std::shared_ptr<ImageContainer> baseContainer)
    : BaseDecorator(baseContainer)
{
}

cv::Mat GrayDecorator::at(std::size_t idx) const
{
    cv::Mat mat = BaseDecorator::at(idx);
    if (mat.channels() != 3)
        return mat;

    cv::Mat grayMat;
    cv::cvtColor(mat, grayMat, cv::ColorConversionCodes::COLOR_BGR2GRAY);
    return grayMat;
}

} // namespace ht
