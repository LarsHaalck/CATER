#ifndef CATER_TRANSFORMATION_H
#define CATER_TRANSFORMATION_H

#include <cater/image-processing/geometricType.h>
#include <cater/image-processing/matches.h>
#include <opencv2/core.hpp>

namespace ct::transformation
{
bool isFull(const cv::Mat& matrix);
cv::Mat makeFull(const cv::Mat& matrix);
cv::Mat invertIsometry(const cv::Mat& matrix, bool full = false);
cv::Mat invertSimilarity(const cv::Mat& matrix, bool full = false);
cv::Mat invertAffine(const cv::Mat& matrix, bool full = false);

cv::Mat invert(const cv::Mat& matrix, GeometricType type, bool full = false);

cv::Mat getIdentity(bool full = false);
cv::Mat getTranslationMat(double tx, double ty, bool full = false);
cv::Mat getScaleMat(double s, bool full = false);

/* cv::Mat concatTo(std::size_t from, std::size_t to, const PairwiseTrafos& trafos, */
/*     GeometricType type, bool full = false); */
/* cv::Mat concatFrom(std::size_t from, std::size_t to, const PairwiseTrafos& trafos, */
/*     GeometricType type, bool full = false); */

std::vector<cv::Point2d> transformPoints(
    const std::vector<cv::Point2d>& points, const cv::Mat& H, GeometricType type);
cv::Point2d transformPoint(const cv::Point2d& refPosition, const cv::Mat& H, GeometricType type);

template <typename T>
std::vector<cv::Point2d> zipTransform(
    const std::vector<cv::Point_<T>>& points, const std::vector<cv::Mat>& Hs, GeometricType type)
{
    assert((points.size() == Hs.size() || points.size() == 1)
        && "Size of points and trafos does not match or points size is not 1 in zipTransform");

    std::vector<cv::Point2d> ptsTrans(Hs.size());
    if (points.size() != 1)
    {
        for (std::size_t i = 0; i < Hs.size(); i++)
            ptsTrans[i] = transformPoint(points[i], Hs[i], type);
    }
    else
    {
        for (std::size_t i = 0; i < Hs.size(); i++)
            ptsTrans[i] = transformPoint(points[0], Hs[i], type);
    }

    return ptsTrans;
}

namespace detail
{
    inline cv::Mat getFullStub(int type)
    {
        cv::Mat mat = cv::Mat(3, 3, type);
        mat.at<double>(2, 0) = mat.at<double>(2, 1) = 0.0;
        mat.at<double>(2, 2) = 1.0;
        return mat;
    }
} // namespace detail
} // namespace ct
#endif // CATER_TRANSFORMATION_H
