#ifndef HABITRACK_TRANSFORMATION_H
#define HABITRACK_TRANSFORMATION_H

#include "habitrack/geometricType.h"
#include <opencv2/core.hpp>

namespace ht
{
cv::Mat makeFull(const cv::Mat& matrix);
cv::Mat invertIsometry(const cv::Mat& matrix, bool full = true);
cv::Mat invertSimilarity(const cv::Mat& matrix, bool full = true);
cv::Mat invertAffine(const cv::Mat& matrix, bool full = true);
cv::Mat invertSpecial(const cv::Mat& matrix, GeometricType type, bool full = true);

cv::Mat getIdentity(bool full = true);
cv::Mat getTranslationMat(double tx, double ty, bool full = true);
cv::Mat getScaleMat(double s, bool full = true);

} // namespace ht
#endif // HABITRACK_TRANSFORMATION_H
