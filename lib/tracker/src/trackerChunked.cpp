#include "tracker/tracker.h"

#include "image-processing/transformation.h"
#include "image-processing/util.h"
#include "io/matndIO.h"
#include "spdlog/spdlog.h"
#include "tracker/manualUnaries.h"
#include "tracker/unaries.h"
#include "util/algorithm.h"
#include "util/threadPool.h"
#include <chrono>
#include <future>
#include <iostream>
#include <limits>
#include <opencv2/imgproc.hpp>

#ifdef HAS_OPENCL
#include "tracker/trackerOpenCL.h"
#endif

namespace ht
{
using namespace matches;
using namespace transformation;
namespace fs = std::filesystem;
using MessagePassing = void (*)(const cv::Mat&, const cv::Mat&, cv::Mat&, cv::Mat&);

// explicit template instantation to allow template function defintion in this unit
template cv::Mat Tracker::truncatedMaxSum<MessagePassing>(std::size_t, std::size_t,
    const std::vector<std::size_t>&, const Unaries&, const ManualUnaries&, const Settings&,
    const cv::Mat&, MessagePassing, const fs::path&);

#ifdef HAS_OPENCL
template cv::Mat Tracker::truncatedMaxSum<TrackerOpenCL>(std::size_t, std::size_t,
    const std::vector<std::size_t>&, const Unaries&, const ManualUnaries&, const Settings&,
    const cv::Mat&, TrackerOpenCL, const fs::path&);
#endif

namespace
{
    void passMessageToNode(const cv::Mat& previousMessageToFactor,
        const cv::Mat& logPairwisePotential, cv::Mat& messageToNode, cv::Mat& phi);
}

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

    auto numChunks = util::getNumChunks(numUnaries, chunkSize);
    spdlog::debug("Single chunk tracking of chunk {}", chunk);

    auto start = std::chrono::system_clock::now();
    std::size_t boundary = util::getChunkEnd(chunk, numChunks, chunkSize, numUnaries);
    auto states = truncatedMaxSum(chunk * chunkSize, boundary, ids, unaries, manualUnaries,
        settings, pairwiseKernel, passMessageToNode, "");
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    spdlog::debug("Elapsed time for chunk {} tracking: {}", chunk, elapsed.count());
    return extractFromStates(states, ids, chunk * chunkSize, settings, trafos);
}

// TODO: maybe remove this function because of redundancy with the function above
Detections Tracker::trackChunked(const Unaries& unaries, const ManualUnaries& manualUnaries,
    const Settings& settings, const PairwiseTrafos& trafos)
{
    auto pairwiseKernel = getPairwiseKernel(settings.pairwiseSize, settings.pairwiseSigma);
    cv::log(pairwiseKernel, pairwiseKernel);
    auto ids = unaries.getIDs();
    auto numUnaries = unaries.size();

    auto chunkSize = settings.chunkSize;
    if (chunkSize == 0)
        chunkSize = numUnaries;

    auto numChunks = util::getNumChunks(numUnaries, chunkSize);
    spdlog::debug("Number of unaries for tracking: {}", numUnaries);
    spdlog::debug("Number of chunks for (parallel) tracking: {}", numChunks);

    std::vector<cv::Mat> states;
    std::vector<std::future<cv::Mat>> futureStates;

    ThreadPool pool;
    auto start = std::chrono::system_clock::now();
    for (std::size_t i = 0; i < numChunks; i++)
    {
        std::size_t end = util::getChunkEnd(i, numChunks, chunkSize, numUnaries);
        auto future = pool.enqueue(truncatedMaxSum<MessagePassing>, i * chunkSize, end, ids,
            unaries, manualUnaries, settings, pairwiseKernel, passMessageToNode, "");
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

template <typename MessagePassing>
cv::Mat Tracker::truncatedMaxSum(std::size_t start, std::size_t end,
    const std::vector<std::size_t>& ids, const Unaries& unaries, const ManualUnaries& manualUnaries,
    const Settings& settings, const cv::Mat& pairwiseKernel, MessagePassing messagePassing,
    const std::filesystem::path& workingDir)
{
    spdlog::debug("Running tracking on boundaries: [{}, {})", start, end);
    spdlog::debug("Running tracking on boundaries (id-space): [{}, {}]", ids[start], ids[end - 1]);

    // number of random variables
    int numVariables = end - start;

    // x,y for every variables
    cv::Mat maxStates(numVariables, 2, CV_32S);
    if (numVariables == 0)
        return maxStates;

    // root node for max sum algorithm
    cv::Mat firstUnary = unaries.at(ids[start]);

    cv::Mat messageToNode = firstUnary.clone();
    cv::log(messageToNode, messageToNode);
    cv::Mat messageToFactor = messageToNode.clone();

    // backtracking data
     std::vector<cv::Mat> phis;
    const int phiSize[] = {firstUnary.rows, firstUnary.cols, 2};

    if (numVariables == 1)
        messageToNode = messageToFactor;

    for (std::size_t i = start; i < end; i++)
    {
        std::size_t idx = ids[i];
        spdlog::debug("Optimize unary {} ({})", i, ids[i]);

        cv::Mat phi(3, phiSize, CV_32S);
        // set message to node by maximizing over log f and message to factor
        messagePassing(messageToFactor, pairwiseKernel, messageToNode, phi);
        if (workingDir.empty())
            phis.push_back(phi);
        else
            savePhi(idx, phi, workingDir);

        cv::Mat currentUnary;
        if (manualUnaries.exists(idx))
            currentUnary = manualUnaries.unaryAt(idx);
        else
            currentUnary = unaries.at(idx);

        cv::log(currentUnary, currentUnary);

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
            cv::minMaxLoc(currentUnary, &tempMin, &tempMax);
            currentUnary += cv::abs(tempMin);
            double multiplier = settings.manualMultiplier;
            currentUnary *= multiplier;
            currentUnary -= cv::abs(tempMin);
        }

        // pass message to factor by simple summation
        messageToFactor = messageToNode + currentUnary;
    }

    // TODO: clean up this redundant mess here
    // find the max of the last message
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

    // backtrack through phi to fill maxStates
    for (auto v = numVariables - 2; v >= 0; v--)
    {
        auto r = maxStates.at<int>(v + 1, 0);
        auto c = maxStates.at<int>(v + 1, 1);
        cv::Mat phi;
        if (workingDir.empty())
            phi = phis[v];
        else
            phi = loadPhi(v, workingDir);
        maxStates.at<int>(v, 0) = phi.at<int>(r, c, 0);
        maxStates.at<int>(v, 1) = phi.at<int>(r, c, 1);
    }

    if (!workingDir.empty())
        fs::remove_all(workingDir / "phis");
    return maxStates;
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

namespace
{
    void passMessageToNode(const cv::Mat& previousMessageToFactor,
        const cv::Mat& logPairwisePotential, cv::Mat& messageToNode, cv::Mat& phi)
    {
        int pairwiseSize = logPairwisePotential.rows;
        int offset = std::floor(pairwiseSize / 2);

        // find max value in truncation window
        for (int i = 0; i < previousMessageToFactor.rows; i++)
        {
            for (int j = 0; j < previousMessageToFactor.cols; j++)
            {
                auto curMax = std::numeric_limits<float>::lowest();
                auto curMaxI = i;
                auto curMaxJ = j;

                // truncation for better performance
                // based on simple heurisitic on pairwise potential size
                for (int ii = -offset; ii <= offset; ii++)
                {
                    auto curI = i + ii;
                    if ((curI < 0) || (curI >= previousMessageToFactor.rows))
                    {
                        continue;
                    }
                    for (int jj = -offset; jj <= offset; jj++)
                    {
                        auto curJ = j + jj;
                        if ((curJ < 0) || (curJ >= previousMessageToFactor.cols))
                        {
                            continue;
                        }

                        // max value is the previous message + the logged function
                        auto curVal = previousMessageToFactor.at<float>(curI, curJ)
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

                // store max indices for later backtracking
                phi.at<int>(i, j, 0) = curMaxI;
                phi.at<int>(i, j, 1) = curMaxJ;
            }
        }
    }
} // unnamed namespace
} // namespace ht
