#include "habitrack/transformation.h"

namespace ht
{
cv::Mat makeFull(const cv::Mat& matrix)
{
    if ((matrix.rows == 3 && matrix.cols == 3)
        || matrix.rows != 2 || matrix.cols != 3)
        return matrix;

    cv::Mat full;
    full = cv::Mat(3, 3, CV_64F);
    full.at<double>(2, 0) = full.at<double>(2, 1) = 0.0;
    full.at<double>(2, 2) = 1.0;

    // copy upper left 2x2 matrix to new matrix
    cv::Mat tmp = full(cv::Rect(0, 0, 2, 2));
    matrix.rowRange(0, 2).colRange(0, 2).copyTo(tmp);
    return full;
}
cv::Mat invertIsometry(const cv::Mat& matrix, bool full)
{
    cv::Mat inverse;
    if (full)
    {
        inverse = cv::Mat(3, 3, matrix.type());
        inverse.at<double>(2, 0) = inverse.at<double>(2, 1) = 0.0;
        inverse.at<double>(2, 2) = 1.0;
    }
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

// TODO this can be done better
cv::Mat invertSimilarity(const cv::Mat& matrix, bool full)
{
    return invertAffine(matrix, full);
}

cv::Mat invertAffine(const cv::Mat& matrix, bool full)
{
    cv::Mat inverse;
    if (full)
    {
        inverse = cv::Mat(3, 3, matrix.type());
        inverse.at<double>(2, 0) = inverse.at<double>(2, 1) = 0.0;
        inverse.at<double>(2, 2) = 1.0;
    }
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

cv::Mat invertSpecial(const cv::Mat& matrix, GeometricType type, bool full)
{
    assert(matrix.type() == CV_64F &&
        "Matrix type not CV_64F in invertSpecial");

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

cv::Mat getTranslationMat(double tx, double ty, double full)
{
    cv::Mat mat;
    if (full)
    {
        mat = cv::Mat::eye(3, 3, CV_64F);
    }
    else
        mat = cv::Mat::eye(2, 3, CV_64F);

    mat.at<double>(0, 2) = tx;
    mat.at<double>(1, 2) = ty;
    return mat;
}

cv::Mat getScaleMat(double scale, double full)
{
    cv::Mat mat;
    if (full)
    {
        mat = cv::Mat::eye(3, 3, CV_64F);
    }
    else
        mat = cv::Mat::eye(2, 3, CV_64F);

    mat.at<double>(0, 0) = mat.at<double>(1, 1) = scale;

    return mat;
}
} // namespace ht
