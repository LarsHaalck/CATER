#include "habitrack/specialGrayDecorator.h"

#include <opencv2/imgproc.hpp>

namespace ht
{
SpecialGrayDecorator::SpecialGrayDecorator(double red, double green, double blue,
    std::shared_ptr<ImageContainer> baseContainer)
    : BaseDecorator(baseContainer)
    , mRed(red)
    , mGreen(green)
    , mBlue(blue)
{
}

cv::Mat SpecialGrayDecorator::at(std::size_t idx) const
{
    cv::Mat mat = BaseDecorator::at(idx);
    if (mat.channels() != 3)
        return mat;

    cv::Mat grayMat = cv::Mat(mat.rows, mat.cols, CV_8U);

    cv::Mat channels[3];
    cv::split(mat, channels);
    channels[0].convertTo(channels[0], CV_32FC1);
    channels[1].convertTo(channels[1], CV_32FC1);
    channels[2].convertTo(channels[2], CV_32FC1);

    channels[0] *= mBlue;
    channels[1] *= mGreen;
    channels[2] *= mRed;

    cv::Mat tmpFrame;
    cv::merge(channels, 3, tmpFrame);
    tmpFrame.convertTo(grayMat, CV_8U);
    return grayMat;
}

} // namespace ht
