#include "image-processing/util.h"

#include "fitpackpp/BSplineCurve.h"
#include "image-processing/idTranslator.h"
#include "util/tinycolormap.h"

#include <opencv2/imgproc.hpp>

constexpr double pi() { return std::atan(1) * 4; }

namespace ht::util
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
    return std::fmod(angle + 5 * pi() / 2, 2 * pi()) * 180 / pi();
}

cv::Point rotatePointAroundPoint(cv::Point center_point, double angle, int radius)
{
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

    cv::Point rotation_point(center_point.x, center_point.y + radius);
    double radian_angle = angle - 180;
    radian_angle *= CV_PI / 180;

    double new_x = center_point.x + (rotation_point.x - center_point.x) * cos(radian_angle)
        - (rotation_point.y - center_point.y) * sin(radian_angle);
    double new_y = center_point.y + (rotation_point.x - center_point.x) * sin(radian_angle)
        + (rotation_point.y - center_point.y) * cos(radian_angle);

    return cv::Point(static_cast<int>(new_x), static_cast<int>(new_y));
}

void highlightImg(cv::Mat& img)
{
    for (int r = 0; r < img.rows; r++)
    {
        for (int c = 0; c < img.cols; c++)
        {
            img.at<cv::Vec3b>(r, c)[1] += 50;
        }
    }
}

cv::Mat overlayPoints(
    const cv::Mat& img, const std::vector<cv::Point>& pts, const std::vector<std::size_t>& sizes)
{
    cv::Mat pano = img.clone();
    namespace tcm = tinycolormap;
    auto Viridis = tcm::ColormapType::Viridis;

    Translator translator;
    if (!sizes.empty())
        translator = Translator(sizes);

    for (std::size_t i = 0; i < pts.size(); i++)
    {
        tcm::Color tcm_color(0, 0, 0);
        if (!sizes.empty())
        {
            auto vidId = translator.globalToLocal(i).first;
            tcm_color = tcm::GetColor(static_cast<double>(vidId) / (sizes.size() - 1), Viridis);
        }
        else
        {
            tcm_color = tcm::GetColor(static_cast<double>(i) / (pts.size() - 1), Viridis);
        }

        auto color = cv::Scalar(tcm_color.b() * 255, tcm_color.g() * 255, tcm_color.r() * 255);
        if (!sizes.empty() && translator.globalToLocal(i).second > 0)
            cv::line(pano, pts[i - 1], pts[i], color, 4);
        else if (sizes.empty() && i > 0)
            cv::line(pano, pts[i - 1], pts[i], color, 4);
    }
    return pano;
}

namespace detail
{
    std::vector<double> getSimpleWeights(int w)
    {
        double k = 3;
        auto weights = std::vector<double>(w);
        std::generate(std::begin(weights), std::end(weights),
            [n = w, k]() mutable { return std::pow((0.9 * n--), k); });
        weights.reserve(2 * w);
        weights.insert(std::end(weights), std::rbegin(weights), std::rend(weights));
        return weights;
    }

    std::vector<double> getCauchyWeights(int w)
    {
        double gamma = std::sqrt(1 / (std::exp(10) - 1));
        double sigma = 1;

        gamma = std::pow(gamma, 2);
        sigma = 2 * std::pow(sigma, 2);

        auto x = std::vector<double>(2 * w);
        std::iota(std::begin(x), std::end(x), -w);

        std::vector<double> weights(2 * w);
        std::transform(std::begin(x), std::end(x), std::begin(weights), [gamma, sigma](double x) {
            return std::log(1 + 1 / gamma)
                - std::log(1 + (std::exp(-(std::pow(x, 2) / (sigma))) / gamma));
        });

        return weights;
    }
} // namespace detail

std::vector<cv::Point> smoothBoundaries(const std::vector<cv::Point>& pts, std::size_t boundarySize)
{
    std::size_t w = 10;

    using namespace fitpackpp;
    std::vector<double> x;
    std::vector<double> y;
    x.reserve(pts.size());
    y.reserve(pts.size());

    for (const auto& pt : pts)
    {
        x.push_back(pt.x);
        y.push_back(pt.y);
    }

    auto ptsSmoothed = pts;
    auto cs = boundarySize;
    auto numUns = pts.size();
    auto numChunks = getNumChunks(numUns, cs);
    for (std::size_t i = 1; i < numChunks; i++)
    {
        auto prevEnd = getChunkEnd(i - 1, numChunks, cs, numUns);

        auto ids = std::vector<double>(2 * w);
        std::iota(std::begin(ids), std::end(ids), prevEnd - w);

        /* auto weights = getSimpleWeights(w); */
        auto weights = detail::getCauchyWeights(w);

        auto splX = BSplineCurve::splrep(ids,
            Vec(std::begin(x) + ids.front(), std::begin(x) + ids.back() + 1), weights, {}, {}, 3);
        auto splY = BSplineCurve::splrep(ids,
            Vec(std::begin(y) + ids.front(), std::begin(y) + ids.back() + 1), weights, {}, {}, 3);

        auto xSmoothed = splX(ids);
        auto ySmoothed = splY(ids);

        std::transform(std::begin(xSmoothed), std::end(xSmoothed), std::begin(ySmoothed),
            std::begin(ptsSmoothed) + ids.front(),
            [](double x, double y) { return cv::Point(std::round(x), std::round(y)); });
    }

    return ptsSmoothed;
}

std::size_t getNumChunks(std::size_t size, std::size_t chunkSize)
{
    if (chunkSize == 0)
        chunkSize = size;

    auto numChunks = std::max(size / chunkSize, static_cast<std::size_t>(1));
    return numChunks;
}

std::size_t getChunkEnd(
    std::size_t chunk, std::size_t numChunks, std::size_t chunkSize, std::size_t size)
{
    if (chunk == numChunks - 1)
        return size;
    return std::min(size, (chunk + 1) * chunkSize);
}

} // namespace ht
