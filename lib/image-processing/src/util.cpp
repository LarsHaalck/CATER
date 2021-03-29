#include "image-processing/util.h"

constexpr double pi() { return std::atan(1)*4; }

namespace ht
{
float gauss1DPDF(float mean, float sigma, float x)
{
    float z = std::pow(x - mean, 2);
    float n = static_cast<float>(2) * std::pow(sigma, 2);
    return z / n;
}

float scaledGauss2DPDF(
    float meanX, float meanY, float sigmaX, float sigmaY, float scale, float x, float y)
{
    float s1 = gauss1DPDF(meanX, sigmaX, x);
    float s2 = gauss1DPDF(meanY, sigmaY, y);
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

double calcAngle(const cv::Point2d& p1, const cv::Point2d& p2)
{
    auto angle = std::atan2(p2.y - p1.y, p2.x - p1.x);
    // atan2 delivers values in [-pi, pi]
    // we have to add do (angle + 2pi) % 2pi to fix this
    // and we have to to (angle + pi/2) % 2pi to rotate the coordinat system
    // --> do (angle + 5pi/2) % 2pi
    return std::fmod(angle + 5*pi()/2, 2*pi()) * 180/pi();
}

cv::Point rotatePointAroundPoint(cv::Point center_point, double angle)
{
    int y_displacement = 20;
    /*
     * https://stackoverflow.com/a/12161405/5924858
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
    // 2) angle in ]180, 360]   -> angle = 360 - angle (so that counter clockwise rotation is in [0,180])
    // 3) convert to radian (*pi/180)

    cv::Point rotation_point(center_point.x, center_point.y + y_displacement);
    double radian_angle = angle - 180;
    radian_angle *= CV_PI / 180;

    double new_x = center_point.x + (rotation_point.x - center_point.x) * cos(radian_angle)
        - (rotation_point.y - center_point.y) * sin(radian_angle);
    double new_y = center_point.y + (rotation_point.x - center_point.x) * sin(radian_angle)
        + (rotation_point.y - center_point.y) * cos(radian_angle);

    return cv::Point(static_cast<int>(new_x), static_cast<int>(new_y));
}

} // namespace ht
