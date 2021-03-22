#include "tracker/tracker.h"

#include "tracker/manualUnaries.h"
#include "util/threadPool.h"
#include "tracker/unaries.h"
#include "image-processing/transformation.h"
#include "image-processing/util.h"
#include "spdlog/spdlog.h"
#include "util/algorithm.h"
#include <chrono>
#include <future>
#include <iostream>
#include <limits>
#include <opencv2/imgproc.hpp>

namespace ht
{
using namespace matches;
using namespace transformation;

Detections Tracker::track(const Unaries& unaries, const ManualUnaries& manualUnaries,
    const Settings& settings, std::size_t chunk, const PairwiseTrafos& trafos)
{
    auto pairwiseKernel = getPairwiseKernel(settings.pairwiseSize, settings.pairwiseSigma);
    cv::log(pairwiseKernel, pairwiseKernel);
    auto ids = unaries.getIDs();
    auto numUnaries = unaries.size();

    auto chunkSize = settings.chunkSize;
    if (chunkSize == 0)
        chunkSize = numUnaries;

    auto numChunks = getNumChunks(numUnaries, chunkSize);
    spdlog::debug("Single chunk tracking of chunk {}", chunk);

    auto start = std::chrono::system_clock::now();
    std::size_t boundary = getChunkEnd(chunk, numChunks, chunkSize, numUnaries);
    auto states = truncatedMaxSum(
        chunk * chunkSize, boundary, ids, unaries, manualUnaries, settings, pairwiseKernel);
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    spdlog::debug("Elapsed time for chunk {} tracking: {}", chunk, elapsed.count());
    return extractFromStates(states, ids, chunk * chunkSize, settings, trafos);
}


// TODO: maybe remove this function because of redundancy with the function above
Detections Tracker::track(const Unaries& unaries, const ManualUnaries& manualUnaries,
    const Settings& settings, const PairwiseTrafos& trafos)
{
    auto pairwiseKernel = getPairwiseKernel(settings.pairwiseSize, settings.pairwiseSigma);
    cv::log(pairwiseKernel, pairwiseKernel);
    auto ids = unaries.getIDs();
    auto numUnaries = unaries.size();

    auto chunkSize = settings.chunkSize;
    if (chunkSize == 0)
        chunkSize = numUnaries;

    auto numChunks = getNumChunks(numUnaries, chunkSize);
    spdlog::debug("Number of unaries for tracking: {}", numUnaries);
    spdlog::debug("Number of chunks for (parallel) tracking: {}", numChunks);

    std::vector<cv::Mat> states;
    std::vector<std::future<cv::Mat>> futureStates;

    ThreadPool pool;
    auto start = std::chrono::system_clock::now();
    for (std::size_t i = 0; i < numChunks; i++)
    {
        std::size_t end = getChunkEnd(i, numChunks, chunkSize, numUnaries);
        auto future = pool.enqueue(truncatedMaxSum, i * chunkSize, end, ids, unaries, manualUnaries,
            settings, pairwiseKernel);
        futureStates.push_back(std::move(future));
    }

    // wait for all threads so futures are avaiable
    pool.join();
    for (auto& f : futureStates)
        states.push_back(f.get());

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    spdlog::info("Elapsed time for tracking: {}", elapsed.count());

    cv::Mat bestStates;
    cv::vconcat(states.data(), states.size(), bestStates);
    return extractFromStates(bestStates, ids, 0, settings, trafos);
}

std::size_t Tracker::getNumChunks(std::size_t numUnaries, std::size_t chunkSize)
{
    if (chunkSize == 0)
        chunkSize = numUnaries;

    auto numChunks = std::max(numUnaries / chunkSize, static_cast<std::size_t>(1));
    return numChunks;
}

std::size_t Tracker::getChunkEnd(
    std::size_t chunk, std::size_t numChunks, std::size_t chunkSize, std::size_t numUnaries)
{
    if (chunk == numChunks - 1)
        return numUnaries;
    return std::min(numUnaries, (chunk + 1) * chunkSize);
}

cv::Mat Tracker::getPairwiseKernel(int size, double sigma)
{
    cv::Mat kernelX = cv::getGaussianKernel(size, sigma, CV_32F);
    cv::Mat kernelY = cv::getGaussianKernel(size, sigma, CV_32F);
    return kernelX * kernelY.t();
}

cv::Mat Tracker::truncatedMaxSum(std::size_t start, std::size_t end,
    const std::vector<std::size_t>& ids, const Unaries& unaries, const ManualUnaries& manualUnaries,
    const Settings& settings, const cv::Mat& pairwiseKernel)
{
    spdlog::debug("Running tracking on boundaries: [{}, {})", start, end);
    spdlog::debug("Running tracking on boundaries (id-space): [{}, {}]", ids[start], ids[end - 1]);
    int numVariables = end - start;

    cv::Mat maxStates(numVariables, 2, CV_32S);
    if (numVariables == 0)
        return maxStates;

    cv::Mat firstUnary = unaries.at(ids[start]);

    cv::Mat messageToFactor = firstUnary.clone();
    cv::Mat messageToNode = firstUnary.clone();
    std::vector<cv::Mat> phis;

    int phiSize[3];
    phiSize[0] = firstUnary.rows;
    phiSize[1] = firstUnary.cols;
    phiSize[2] = 2;

    cv::log(messageToFactor, messageToFactor);
    cv::log(messageToNode, messageToNode);

    if (numVariables == 1)
        messageToNode = messageToFactor;

    cv::Mat tempU = firstUnary.clone();
    for (std::size_t i = start; i < end; i++)
    {
        std::size_t idx = ids[i];
        spdlog::debug("Optimize unary {} ({})", i, ids[i]);
        cv::Mat phi(3, phiSize, CV_32S);

        passMessageToNode(messageToFactor, pairwiseKernel, messageToNode, phi);
        phis.push_back(phi);

        cv::Mat currentUnary;
        if (manualUnaries.exists(idx))
            currentUnary = manualUnaries.unaryAt(idx);
        else
            currentUnary = unaries.at(idx);

        cv::log(currentUnary, tempU);

        // To make manual positions stronger the (logarithmic) unary is multiplied
        // NOTE: due to the log operation the values in the unary are in [-x, +/- y]
        // (y can be positive or negative). To make multiplication possible without
        // enhancing the small values first the absolut minimum is added so that the
        // inverval is in [0, +/- y + x] (note that +/- y + x > 0), second the values
        // are multiplied and finally the absolut value of x is subtracted again so
        // that the final interval is [-x, z] with 0 < z and |y| < z.
        if (manualUnaries.exists(idx))
        {
            double tempMin, tempMax;
            cv::minMaxLoc(tempU, &tempMin, &tempMax);
            tempU += cv::abs(tempMin);
            double multiplier = settings.manualMultiplier;
            tempU *= multiplier;
            tempU -= cv::abs(tempMin);
        }

        messageToFactor = messageToNode + tempU;
    }

    // Now find the max of the last message
    float logBestConfigurationProbability = std::numeric_limits<float>::lowest();
    float curVal = std::numeric_limits<float>::lowest();
    int curMaxI = 0;
    int curMaxJ = 0;

    for (int i = 0; i < messageToFactor.rows; i++)
    {
        for (int j = 0; j < messageToFactor.cols; j++)
        {
            curVal = messageToFactor.at<float>(i, j);
            if (curVal > logBestConfigurationProbability)
            {
                logBestConfigurationProbability = curVal;
                curMaxI = i;
                curMaxJ = j;
            }
        }
    }
    maxStates.at<int>(numVariables - 1, 0) = curMaxI;
    maxStates.at<int>(numVariables - 1, 1) = curMaxJ;

    for (int v = (numVariables - 2); v >= 0; v--)
    {
        maxStates.at<int>(v, 0)
            = phis[v].at<int>(maxStates.at<int>(v + 1, 0), maxStates.at<int>(v + 1, 1), 0);
        maxStates.at<int>(v, 1)
            = phis[v].at<int>(maxStates.at<int>(v + 1, 0), maxStates.at<int>(v + 1, 1), 1);
    }
    return maxStates;
}

void Tracker::passMessageToNode(const cv::Mat& previousMessageToFactor,
    const cv::Mat& logPairwisePotential, cv::Mat& messageToNode, cv::Mat& phi)
{
    int pairwiseSize = logPairwisePotential.rows;
    int offset = std::floor(pairwiseSize / 2);

    float curMax = std::numeric_limits<float>::lowest();
    float curVal = std::numeric_limits<float>::lowest();
    int curMaxI = 0;
    int curMaxJ = 0;
    int curI = 0;
    int curJ = 0;

    for (int i = 0; i < previousMessageToFactor.rows; i++)
    {
        for (int j = 0; j < previousMessageToFactor.cols; j++)
        {
            curMax = std::numeric_limits<float>::lowest();
            curMaxI = i;
            curMaxJ = j;
            for (int ii = -offset; ii <= offset; ii++)
            {
                curI = i + ii;
                if ((curI < 0) || (curI >= previousMessageToFactor.rows))
                {
                    continue;
                }
                for (int jj = -offset; jj <= offset; jj++)
                {
                    curJ = j + jj;
                    if ((curJ < 0) || (curJ >= previousMessageToFactor.cols))
                    {
                        continue;
                    }

                    curVal = previousMessageToFactor.at<float>(curI, curJ)
                        + logPairwisePotential.at<float>(ii + offset, jj + offset);
                    if (curVal > curMax)
                    {
                        curMax = curVal;
                        curMaxI = curI;
                        curMaxJ = curJ;
                    }
                }
            }
            messageToNode.at<float>(i, j) = curMax;
            phi.at<int>(i, j, 0) = curMaxI;
            phi.at<int>(i, j, 1) = curMaxJ;
        }
    }
}

void Tracker::passMessageToFactor(
    const cv::Mat& previousMessageToNode, const cv::Mat& unaryPotential, cv::Mat& messageToFactor)
{
    cv::log(unaryPotential, messageToFactor);
    cv::add(previousMessageToNode, messageToFactor, messageToFactor);
}

Detections Tracker::extractFromStates(const cv::Mat& states, const std::vector<std::size_t>& ids,
    std::size_t offset, const Settings& settings, const PairwiseTrafos& trafos)
{
    Detections detections;

    std::size_t lastIdx;
    cv::Point lastPos;
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
    auto transLastPos
        = transformPoint(lastPos, trafos.at({lastIdx, idx}), GeometricType::Homography);
    auto theta = calcAngle(transLastPos, pos);
    auto thetaQuality = euclidianDist<double>(transLastPos, pos);
    return {theta, thetaQuality};
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

    // TODO: implement
    /* // set all manually set bearings to 1 (highest probability) */
    /* int vecCounter = 0; */
    /* for (int i = firstFrameUsed; i <= lastFrameUsed; ++i) */
    /* { */
    /*     cv::Point manualBearingPoint; */
    /*     if (trackingData.getManuallySetBearingDirectionPoint(i, manualBearingPoint)) */
    /*     { */
    /*         lengthVec.at(vecCounter) = 1.0; */
    /*     } */
    /*     vecCounter++; */
    /* } */

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
} // namespace ht
