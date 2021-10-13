#include "tracker/tracker.h"

#include "image-processing/transformation.h"
#include "image-processing/util.h"
#include "spdlog/spdlog.h"
#include "tracker/manualUnaries.h"
#include "tracker/unaries.h"
#include "util/algorithm.h"
#include <chrono>
#include <iostream>
#include <limits>
#include <opencv2/imgproc.hpp>

namespace ht
{

namespace
{
    void passMessageToNode(const cv::Mat& previousMessageToFactor,
        const cv::Mat& logPairwisePotential, cv::Mat& messageToNode, cv::Mat& phi);
}

// TODO: maybe remove this function because of redundancy with the function above
Detections Tracker::trackContinous(const Unaries& unaries, const ManualUnaries& manualUnaries,
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

    auto start = std::chrono::system_clock::now();
    auto states = truncatedMaxSum(0, numUnaries, ids, unaries, manualUnaries, settings,
        pairwiseKernel, passMessageToNode, unaries.getUnaryDirectory());
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    spdlog::info("Elapsed time for tracking: {}", elapsed.count());

    return extractFromStates(states, ids, 0, settings, trafos);
}

namespace
{
    void passMessageToNode(const cv::Mat& previousMessageToFactor,
        const cv::Mat& logPairwisePotential, cv::Mat& messageToNode, cv::Mat& phi)
    {
        int pairwiseSize = logPairwisePotential.rows;
        int offset = std::floor(pairwiseSize / 2);

        // find max value in truncation window
#pragma omp parallel for
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
