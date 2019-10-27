#ifndef HABITRACK_TRANSFORMATION_H
#define HABITRACK_TRANSFORMATION_H

#include <opencv2/core.hpp>
#include "habitrack/geometricType.h"

namespace ht
{
cv::Mat makeFull(const cv::Mat& matrix);
cv::Mat invertIsometry(const cv::Mat& matrix, bool full = true);
cv::Mat invertSimilarity(const cv::Mat& matrix, bool full = true);
cv::Mat invertAffine(const cv::Mat& matrix, bool full = true);
cv::Mat invertSpecial(const cv::Mat& matrix, GeometricType type, bool full = true);

cv::Mat getTranslationMat(double tx, double ty, double full = true);
cv::Mat getScaleMat(double s, double full = true);
} // namespace ht
#endif // HABITRACK_TRANSFORMATION_H
