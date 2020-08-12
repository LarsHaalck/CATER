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

double calcAngle(const cv::Point& p, const cv::Point& p2)
{
    cv::Point p1(p.x, 0);
    cv::Point l1 = p1 - p;
    cv::Point l2 = p2 - p;

    double norm1 = normL2(l1);
    double norm2 = normL2(l2);

    double testForNan = (l1.x * l2.x + l1.y * l2.y) / (norm1 * norm2);

    double angle;

    if (testForNan >= 1)
        angle = 0.0;
    else if (testForNan <= -1)
        angle = 180.0;
    else
        angle = (acos((l1.x * l2.x + l1.y * l2.y) / (norm1 * norm2)) * 180 / CV_PI);

    double sgn = (l1.x * l2.y) - (l1.y * l2.x);

    if (sgn < 0)
        angle = 360 - angle;
    return angle;
}

cv::Point rotatePointAroundPoint(cv::Point center_point, double angle)
{
    int y_displacement = 20;
    /*
     * The easiest approach is to compose three transformations:
     *
     * 1. A translation that brings point 1 to the origin
     * 2. Rotation around the origin by the required angle
     * 3. A translation that brings point 1 back to its original position
     *
     * When you work this all out, you end up with the following transformation:
     *
     * newX = centerX + (point2x-centerX)*Math.cos(x) - (point2y-centerY)*Math.sin(x);
     * newY = centerY + (point2x-centerX)*Math.sin(x) + (point2y-centerY)*Math.cos(x);
     *
     * Note that this makes the standard assumtion that the angle x is negative for
     * clockwise rotation. If that's not the case, then you would need to reverse the sign
     * on the terms involving sin(x).
     */

    // currently angle is in degree with 0° up, 90° left, 180° down and 270° right.
    // to convert:
    // 1) angle in [0,180]      -> make negative [-0, -180]   (clockwise rotation)
    // 2) angle in ]180, 360]   -> angle = 360 - angle (so that counter clockwise rotation
    // is in [0,180]) 3) convert to radian (*pi/180)

    cv::Point rotation_point(center_point.x, center_point.y + y_displacement);
    double radian_angle = angle - 180;
    radian_angle *= CV_PI / 180; // 3)

    double new_x = center_point.x + (rotation_point.x - center_point.x) * cos(radian_angle)
        - (rotation_point.y - center_point.y) * sin(radian_angle);
    double new_y = center_point.y + (rotation_point.x - center_point.x) * sin(radian_angle)
        + (rotation_point.y - center_point.y) * cos(radian_angle);

    return cv::Point(static_cast<int>(new_x), static_cast<int>(new_y));
}

} // namespace ht
