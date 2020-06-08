#include "image-processing/transformation.h"

namespace ht::transformation
{

bool isFull(const cv::Mat& matrix)
{
    return ((matrix.rows == 3 && matrix.cols == 3) || matrix.rows != 2 || matrix.cols != 3);
}

cv::Mat makeFull(const cv::Mat& matrix)
{
    if (isFull(matrix))
        return matrix;

    cv::Mat full = detail::getFullStub(CV_64F);

    // copy upper left 2x2 matrix to new matrix
    cv::Mat tmp = full(cv::Rect(0, 0, 3, 2));
    matrix.rowRange(0, 2).colRange(0, 3).copyTo(tmp);
    return full;
}
cv::Mat invertIsometry(const cv::Mat& matrix, bool full)
{
    cv::Mat inverse;
    if (full)
        inverse = detail::getFullStub(matrix.type());
    else
        inverse = cv::Mat(2, 3, matrix.type());

    // copy upper left 2x2 matrix to new matrix
    cv::Mat tmp = inverse(cv::Rect(0, 0, 2, 2));
    matrix.rowRange(0, 2).colRange(0, 2).copyTo(tmp);

    // invert (transpose in this case)) upper left 2x2
    cv::Mat subMat = inverse.rowRange(0, 2).colRange(0, 2);
    subMat = subMat.t();

    // -A^T * t for translation part
    inverse.rowRange(0, 2).col(2) = -subMat * matrix.rowRange(0, 2).col(2);
    return inverse;
}

cv::Mat invertSimilarity(const cv::Mat& matrix, bool full)
{
    cv::Mat inverse;
    if (full)
        inverse = detail::getFullStub(matrix.type());
    else
        inverse = cv::Mat(2, 3, matrix.type());

    // copy upper left 2x2 matrix to new matrix
    cv::Mat tmp = inverse(cv::Rect(0, 0, 2, 2));
    matrix.rowRange(0, 2).colRange(0, 2).copyTo(tmp);

    // invert (transpose in this case)) upper left 2x2 and rescale it
    auto scale = cv::norm(matrix.col(0));
    scale *= scale;
    cv::Mat subMat = inverse.rowRange(0, 2).colRange(0, 2);
    subMat = 1 / scale * subMat.t();

    // -A^T * t for translation part
    inverse.rowRange(0, 2).col(2) = -subMat * matrix.rowRange(0, 2).col(2);
    return inverse;
}

cv::Mat invertAffine(const cv::Mat& matrix, bool full)
{
    cv::Mat inverse;
    if (full)
        inverse = detail::getFullStub(matrix.type());
    else
        inverse = cv::Mat(2, 3, matrix.type());

    // copy upper left 2x2 matrix to new matrix
    cv::Mat tmp = inverse(cv::Rect(0, 0, 2, 2));
    matrix.rowRange(0, 2).colRange(0, 2).copyTo(tmp);

    // invert upper left 2x2
    cv::Mat subMat = inverse.rowRange(0, 2).colRange(0, 2);
    subMat = subMat.inv();

    // -A^T * t for translation part
    inverse.rowRange(0, 2).col(2) = -subMat * matrix.rowRange(0, 2).col(2);

    return inverse;
}

cv::Mat invert(const cv::Mat& matrix, GeometricType type, bool full)
{
    switch (type)
    {
    case GeometricType::Isometry:
        return invertIsometry(matrix, full);
    case GeometricType::Similarity:
        return invertSimilarity(matrix, full);
    case GeometricType::Affinity:
        return invertAffine(matrix, full);
    case GeometricType::Homography:
    {
        cv::Mat inverse;
        cv::invert(matrix, inverse);
        return inverse;
    }
    default:
        break;
    }

    return cv::Mat();
}

cv::Mat getIdentity(bool full)
{
    cv::Mat mat;
    if (full)
        mat = cv::Mat::eye(3, 3, CV_64F);
    else
        mat = cv::Mat::eye(2, 3, CV_64F);

    return mat;
}

cv::Mat getTranslationMat(double tx, double ty, bool full)
{
    cv::Mat mat = getIdentity(full);
    mat.at<double>(0, 2) = tx;
    mat.at<double>(1, 2) = ty;
    return mat;
}

cv::Mat getScaleMat(double scale, bool full)
{
    cv::Mat mat = getIdentity(full);
    mat.at<double>(0, 0) = mat.at<double>(1, 1) = scale;
    return mat;
}

std::vector<cv::Point2d> transformPoints(
    const std::vector<cv::Point2d>& points, const cv::Mat& H, GeometricType type)
{
    std::vector<cv::Point2d> inPts;
    std::vector<cv::Point2d> outPts;

    std::transform(points.begin(), points.end(), std::back_inserter(inPts),
        [](cv::Point pt) -> cv::Point2d { return pt; });

    if (type == GeometricType::Homography)
        cv::perspectiveTransform(inPts, outPts, H);
    else
    {
        auto HFull = makeFull(H);
        cv::perspectiveTransform(inPts, outPts, H);
    }
    return outPts;
}

cv::Point2d transformPoint(const cv::Point2d& refPosition, const cv::Mat& H, GeometricType type)
{
    return transformPoints({refPosition}, H, type)[0];
}
/* cv::Mat concatTo(std::size_t from, std::size_t to, const matches::PairwiseTrafos& trafos, */
/*     GeometricType type, bool full) */
/* { */
/*     assert(from < to && "from should be smaller than to in concatTo()"); */
/*     auto trafo = makeFull(trafos.at({from, from + 1})); */
/*     for (std::size_t i = from + 1; i < to; i++) */
/*     { */
/*         auto currTrafo = makeFull(trafos.at({i, i + 1})); */
/*         trafo = trafo * currTrafo; */
/*     } */

/*     if (type == GeometricType::Homography || full) */
/*         return trafo; */
/*     cv::Mat upper = trafo(cv::Rect(0, 0, 3, 2)); */
/*     return upper; */
/* } */

/* cv::Mat concatFrom(std::size_t from, std::size_t to, const matches::PairwiseTrafos& trafos, */
/*     GeometricType type, bool full) */
/* { */
/*     assert(from < to && "from should be smaller than to in concatFrom()"); */
/*     return invert(concatTo(from, to, trafos, type, true), type, full); */
/* } */
} // namespace ht
