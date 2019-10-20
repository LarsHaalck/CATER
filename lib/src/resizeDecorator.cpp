#include "habitrack/resizeDecorator.h"

#include <opencv2/imgproc.hpp>

namespace ht
{
ResizeDecorator::ResizeDecorator(double scaleX, double scaleY,
    std::shared_ptr<ImageContainer> baseContainer)
    : BaseDecorator(baseContainer)
    , mScaleX(scaleX)
    , mScaleY(scaleY)
{
}

cv::Mat ResizeDecorator::at(std::size_t idx, ImageType imageType) const
{
    cv::Mat mat = BaseDecorator::at(idx, imageType);

    cv::Mat resizedMat;
    cv::resize(mat, resizedMat, cv::Size(), mScaleX, mScaleY,
        cv::InterpolationFlags::INTER_CUBIC);
    return resizedMat;
}
} // namespace ht
