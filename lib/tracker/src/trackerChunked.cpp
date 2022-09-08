#include <cater/tracker/tracker.h>

#include <cater/image-processing/transformation.h>
#include <cater/image-processing/util.h>
#include <cater/tracker/manualUnaries.h>
#include <cater/tracker/unaries.h>
#include <cater/util/algorithm.h>
#include <cater/util/threadPool.h>

#include <spdlog/spdlog.h>

#include <chrono>
#include <future>
#include <iostream>
#include <limits>
#include <opencv2/imgproc.hpp>

namespace ct
{
using namespace matches;
using namespace transformation;
namespace fs = std::filesystem;

namespace
{
    void passMessageToNode(const cv::Mat& previousMessageToFactor,
        const cv::Mat& logPairwisePotential, cv::Mat& messageToNode, cv::Mat& phi);
    cv::Mat truncatedMaxSum(std::size_t start, std::size_t end, const std::vector<std::size_t>& ids,
        const Unaries& unaries, const ManualUnaries& manualUnaries,
        const Tracker::Settings& settings, const cv::Mat& pairwiseKernel,
        const std::filesystem::path& workingDir);
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
    auto states = truncatedMaxSum(
        chunk * chunkSize, boundary, ids, unaries, manualUnaries, settings, pairwiseKernel, "");
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

    /* auto pool = ThreadPool(4); */
    auto pool = ThreadPool(8);
    auto start = std::chrono::system_clock::now();
    for (std::size_t i = 0; i < numChunks; i++)
    {
        std::size_t end = util::getChunkEnd(i, numChunks, chunkSize, numUnaries);
        auto future = pool.enqueue(truncatedMaxSum, i * chunkSize, end, ids, unaries, manualUnaries,
            settings, pairwiseKernel, "");
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

namespace
{
    cv::Mat truncatedMaxSum(std::size_t start, std::size_t end, const std::vector<std::size_t>& ids,
        const Unaries& unaries, const ManualUnaries& manualUnaries,
        const Tracker::Settings& settings, const cv::Mat& pairwiseKernel,
        const std::filesystem::path& workingDir)
    {
        spdlog::debug("Running tracking on boundaries: [{}, {})", start, end);
        spdlog::debug(
            "Running tracking on boundaries (id-space): [{}, {}]", ids[start], ids[end - 1]);

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
            passMessageToNode(messageToFactor, pairwiseKernel, messageToNode, phi);
            if (workingDir.empty())
                phis.push_back(phi);
            else
                Tracker::savePhi(idx, phi, workingDir);

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
                phi = Tracker::loadPhi(ids[v], workingDir);
            maxStates.at<int>(v, 0) = phi.at<int>(r, c, 0);
            maxStates.at<int>(v, 1) = phi.at<int>(r, c, 1);
        }

        if (!workingDir.empty())
            fs::remove_all(workingDir / "phis");
        return maxStates;
    }

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
} // namespace ct
