#include "habitrack/tracker.h"

#include "habitrack/manualUnaries.h"
#include "habitrack/unaries.h"
#include "spdlog/spdlog.h"
#include <future>
#include <iostream>
#include <limits>
#include <opencv2/imgproc.hpp>
#include "habitrack/threadPool.h"

#include <chrono>


namespace ht
{
using Task = std::packaged_task<cv::Mat(std::size_t, std::size_t, const std::vector<std::size_t>&,
    const Unaries&, const ManualUnaries&, Tracker::UnarySettings unarySettings, const cv::Mat&)>;

Detections Tracker::track(const Unaries& unaries, const ManualUnaries& manualUnaries,
    UnarySettings unarySettings, SmoothBearingSettings smoothBearingSettings, std::size_t chunkSize)
{
    auto pairwiseKernel
        = getPairwiseKernel(unarySettings.pairwiseSize, unarySettings.pairwiseSigma);
    cv::log(pairwiseKernel, pairwiseKernel);
    auto ids = unaries.getIDs();
    auto numUnaries = unaries.size();

    if (chunkSize == 0)
        chunkSize = numUnaries;

    spdlog::debug("Number of unaries for tracking: {}", numUnaries);
    auto numChunks = std::max(numUnaries / chunkSize, static_cast<std::size_t>(1));
    spdlog::debug("Number of chunks for (parallel) tracking: {}", numChunks);

    std::vector<cv::Mat> states;
    std::vector<std::future<cv::Mat>> futureStates;

    auto maxThreads = std::thread::hardware_concurrency();
    ThreadPool pool(maxThreads);

    auto start = std::chrono::system_clock::now();
    for (std::size_t i = 0; i < numChunks; i++)
    {
        std::size_t end;
        if (i == numChunks - 1)
            end = numUnaries;
        else
            end = std::min(numUnaries, (i + 1) * chunkSize);

        auto future = pool.enqueue(truncatedMaxSum, i * chunkSize, end, ids, unaries, manualUnaries,
            unarySettings, pairwiseKernel);
        futureStates.push_back(std::move(future));
    }

    // wait for all threads so futures are avaiable
    pool.join();
    for (auto& f : futureStates)
        states.push_back(f.get());

    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    spdlog::critical("Elapsed: {}", elapsed.count());

    cv::Mat bestStates;
    cv::vconcat(states.data(), states.size(), bestStates);

    Detections detections;
    for (int row = 0; row < bestStates.rows; ++row)
    {
        // get best x and y position
        int best_state_x = bestStates.at<int>(row, 1);
        int best_state_y = bestStates.at<int>(row, 0);

        // resample x and y position (due to downsampling)
        double up_sampling_factor = 1.0 / unarySettings.subsample;
        best_state_x *= up_sampling_factor;
        best_state_y *= up_sampling_factor;

        std::size_t idx = ids[row];

        // resultant ant position
        cv::Point antPosition(best_state_x, best_state_y);

        // TODO: implement
        double theta = 0;
        double thetaQuality = 0;
        /* calcBearingAndQuality(trackingData, idx, antPosition, theta, thetaQuality); */
        spdlog::debug("extracted detection {}/{}, theta {} with quality {}", row,
            bestStates.rows - 1, theta, thetaQuality);

        detections.insert(idx, {antPosition, theta, thetaQuality});
    }
    return detections;
}

cv::Mat Tracker::getPairwiseKernel(int size, double sigma)
{
    cv::Mat kernelX = cv::getGaussianKernel(size, sigma, CV_32F);
    cv::Mat kernelY = cv::getGaussianKernel(size, sigma, CV_32F);
    return kernelX * kernelY.t();
}

cv::Mat Tracker::truncatedMaxSum(std::size_t start, std::size_t end,
    const std::vector<std::size_t>& ids, const Unaries& unaries, const ManualUnaries& manualUnaries,
    UnarySettings unarySettings, const cv::Mat& pairwiseKernel)
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
            double multiplier = unarySettings.manualMultiplier;
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
} // namespace ht
