#include <cater/tracker/tracker.h>

#include <cater/image-processing/transformation.h>
#include <cater/image-processing/util.h>
#include <cater/io/matndIO.h>
#include <cater/tracker/manualUnaries.h>
#include <cater/tracker/unaries.h>
#include <cater/util/algorithm.h>

#include <chrono>
#include <future>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace ct
{
/* using namespace matches; */
using namespace transformation;
namespace fs = std::filesystem;

std::ostream& operator<<(std::ostream& stream, const Tracker::Settings& settings)
{
    stream << "subsample: " << settings.subsample << "\n";
    stream << "pairwiseSize: " << settings.pairwiseSize << "\n";
    stream << "pairwiseSigma: " << settings.pairwiseSigma << "\n";
    stream << "manualMultiplier: " << settings.manualMultiplier << "\n";
    stream << "smoothBearing: " << settings.smoothBearing << "\n";
    stream << "windowSize: " << settings.windowSize << "\n";
    stream << "outlierTolerance: " << settings.outlierTolerance << "\n";
    stream << "chunkSize: " << settings.chunkSize << "\n";

    return stream;
}

cv::Mat Tracker::getPairwiseKernel(int size, double sigma)
{
    cv::Mat kernelX = cv::getGaussianKernel(size, sigma, CV_32F);
    cv::Mat kernelY = cv::getGaussianKernel(size, sigma, CV_32F);
    return kernelX * kernelY.t();
}

void Tracker::savePhi(std::size_t idx, const cv::Mat& phi, const fs::path& workingDir)
{
    auto file = workingDir / "phis" / (std::string("phi_") + std::to_string(idx));
    fs::create_directory(file.parent_path());
    std::ofstream stream(file.string(), std::ios::out);
    io::checkStream(stream, file);
    {
        cereal::PortableBinaryOutputArchive archive(stream);
        archive(MatND(phi));
    }
}

cv::Mat Tracker::loadPhi(std::size_t idx, const fs::path& workingDir)
{
    auto file = workingDir / "phis" / (std::string("phi_") + std::to_string(idx));
    std::ifstream stream(file.string(), std::ios::in);
    io::checkStream(stream, file);
    MatND phi;
    {
        cereal::PortableBinaryInputArchive archive(stream);
        archive(phi);
    }
    return phi._m;
}

Detections Tracker::extractFromStates(const cv::Mat& states, const std::vector<std::size_t>& ids,
    std::size_t offset, const Settings& settings, const PairwiseTrafos& trafos)
{
    Detections detections;

    std::size_t lastIdx = 0;
    auto lastPos = cv::Point();
    for (int row = 0; row < states.rows; ++row)
    {
        // get best x and y position
        int best_state_x = states.at<int>(row, 1);
        int best_state_y = states.at<int>(row, 0);

        // resample x and y position (due to downsampling)
        double up_sampling_factor = 1.0 / settings.subsample;
        best_state_x *= up_sampling_factor;
        best_state_y *= up_sampling_factor;

        std::size_t idx = ids[row + offset];

        // resultant position
        cv::Point position(best_state_x, best_state_y);

        double theta = -1;
        double thetaQuality = -1;
        if (row > 0)
        {
            auto [theta_, thetaQuality_]
                = calcBearingAndQuality(lastIdx, idx, lastPos, position, trafos);
            theta = theta_;
            thetaQuality = thetaQuality_;
        }

        spdlog::debug("extracted detection {}/{}, theta {} with quality {}", row, states.rows - 1,
            theta, thetaQuality);

        lastIdx = idx;
        lastPos = position;
        detections.insert(idx, {position, theta, thetaQuality});
    }
    if (settings.smoothBearing)
        smoothBearing(detections, settings);

    return detections;
}

std::pair<double, double> Tracker::calcBearingAndQuality(std::size_t lastIdx, std::size_t idx,
    cv::Point lastPos, cv::Point pos, const PairwiseTrafos& trafos)
{
    if (trafos.count({lastIdx, idx}) > 0)
    {
        auto transLastPos
            = transformPoint(lastPos, trafos.at({lastIdx, idx}), GeometricType::Homography);
        auto theta = util::calcAngle(transLastPos, pos);
        auto thetaQuality = util::euclidianDist<double>(transLastPos, pos);
        return {theta, thetaQuality};
    }
    return {0, -1};
}

void Tracker::smoothBearing(Detections& detections, const Settings& settings)
{
    auto& data = detections.data();
    std::vector<double> anglesVec;
    anglesVec.reserve(data.size());

    std::vector<std::size_t> idx;
    idx.reserve(data.size());

    for (const auto& elem : data)
    {
        anglesVec.push_back(elem.second.theta);
        idx.push_back(elem.first);
    }

    std::vector<double> normalisedWeights
        = filterAndNormaliseLengthVec(detections, settings.outlierTolerance);

    std::vector<std::vector<double>> angleWindows = getWindows(anglesVec, settings.windowSize);
    spdlog::debug("angles vec windows size: {}", angleWindows.size());

    std::vector<std::vector<double>> weightWindows
        = getWindows(normalisedWeights, settings.windowSize);
    spdlog::debug("weights vec windows size: {}", weightWindows.size());

    std::vector<double> smoothedAnglesVec;
    for (std::size_t i = 0; i < angleWindows.size(); ++i)
    {
        smoothedAnglesVec.push_back(
            calcWeightedCircularMean(weightWindows.at(i), angleWindows.at(i)));
    }

    for (std::size_t i = 0; i < smoothedAnglesVec.size(); ++i)
    {
        double smoothedAngle = smoothedAnglesVec.at(i);
        auto& elem = data[idx[i]];
        elem.theta = smoothedAngle;
    }
}

std::vector<double> Tracker::filterAndNormaliseLengthVec(
    const Detections& detections, int outlierTolerance)
{
    // get all length values and max length value
    std::vector<double> lengthVec;
    lengthVec.reserve(detections.size());
    double maxVal = 0;
    for (const auto& elem : detections.cdata())
    {
        double length = elem.second.thetaQuality;
        lengthVec.push_back(length);
        if (length >= maxVal)
            maxVal = length;
    }

    // set the outlier threshold to 3*STD
    double meanVec = mean(std::begin(lengthVec), std::end(lengthVec));
    double stdev = std_dev(std::begin(lengthVec), std::end(lengthVec));
    double maxThresh = meanVec + (outlierTolerance * stdev);

    // remove outliers and normalise so that it is in [0,1]
    for (auto it = lengthVec.begin(); it != lengthVec.end(); ++it)
    {
        if (*it >= maxThresh)
        {
            *it = 0;
        }
        *it /= maxVal;
    }

    // set all manually set bearings to 1 (highest probability)
    int i = 0;
    for (const auto& elem : detections.cdata())
    {
        if (detections.manualBearingExists(elem.first))
            lengthVec[i] = 1.0;
        i++;
    }

    return lengthVec;
}

std::vector<std::vector<double>> Tracker::getWindows(
    const std::vector<double>& vec, std::size_t windowSize)
{
    std::vector<std::vector<double>> windows;
    if (vec.size() > windowSize)
    {
        for (std::size_t globalCounter = 0; globalCounter < vec.size() - windowSize;
             ++globalCounter)
        {
            std::vector<double> tmpWindow;
            for (std::size_t innerCounter = globalCounter;
                 innerCounter < globalCounter + windowSize; ++innerCounter)
            {
                tmpWindow.push_back(vec.at(innerCounter));
            }
            windows.push_back(tmpWindow);
        }
    }
    return windows;
}

double Tracker::calcWeightedCircularMean(
    const std::vector<double>& weightsWindow, const std::vector<double>& anglesWindow)
{
    double x = 0.0;
    double y = 0.0;

    for (std::size_t i = 0; i < weightsWindow.size(); ++i)
    {
        double weight = weightsWindow.at(i);
        double angle = anglesWindow.at(i);
        double radianAngle = degree2Radian(angle);

        x += std::cos(radianAngle) * weight;
        y += std::sin(radianAngle) * weight;
    }

    double tmp = std::atan2(y, x);
    return radian2Degree(tmp);
}
} // namespace ct
