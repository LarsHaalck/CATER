#include <habitrack/tracker/tracker.h>

#include <habitrack/image-processing/transformation.h>
#include <habitrack/image-processing/util.h>
#include <habitrack/tracker/manualUnaries.h>
#include <habitrack/tracker/trackerOpenCL.h>
#include <habitrack/tracker/unaries.h>
#include <habitrack/util/algorithm.h>

#include <spdlog/spdlog.h>
#include <chrono>
#include <iostream>
#include <limits>
#include <opencv2/imgproc.hpp>

namespace ht
{
namespace fs = std::filesystem;

const char* TrackerOpenCL::mKernelSource
    = "__kernel void message_passing(\n"
      "    __global const float* prev,\n"
      "    __global const float* pairwise,\n"
      "    __global float* curr,\n"
      "    __global int* ids,\n"
      "    int offset, int rows, int cols)\n"
      "{\n"
      "    int i = get_global_id(0);\n"
      "    int j = get_global_id(1);\n"
      "\n"
      "    // boundary check\n"
      "    if (i >= rows || j >= cols)\n"
      "        return;\n"
      "\n"
      "    float curMax = -INFINITY;\n"
      "    int curMaxI = i;\n"
      "    int curMaxJ = j;\n"
      "\n"
      "    for (int ii = -offset; ii <= offset; ii++)\n"
      "    {\n"
      "        int curI = i + ii;\n"
      "        if ((curI < 0) || (curI >= rows))\n"
      "            continue;\n"
      "        for (int jj = -offset; jj <= offset; jj++)\n"
      "        {\n"
      "            int curJ = j + jj;\n"
      "            if ((curJ < 0) || (curJ >= cols))\n"
      "                continue;\n"
      "\n"
      "            float curPrev = prev[curI * cols + curJ];\n"
      "            float curPP = pairwise[(ii + offset) * cols + (jj + offset)];\n"
      "            float curVal = curPrev + curPP;\n"
      "\n"
      "            if (curVal > curMax)\n"
      "            {\n"
      "                curMax = curVal;\n"
      "                curMaxI = curI;\n"
      "                curMaxJ = curJ;\n"
      "            }\n"
      "        }\n"
      "    }\n"
      "    curr[i * cols + j] = curMax;\n"
      "    vstore2((int2)(curMaxI, curMaxJ), i * cols + j, ids);\n"
      "}\n";

namespace
{
    cv::Mat truncatedMaxSum(std::size_t start, std::size_t end, const std::vector<std::size_t>& ids,
        const Unaries& unaries, const ManualUnaries& manualUnaries,
        const Tracker::Settings& settings, const cv::UMat& pairwiseKernel,
        const std::filesystem::path& workingDir);
}
TrackerOpenCL::TrackerOpenCL()
{
    auto device = cv::ocl::Device::getDefault();
    spdlog::debug("Using OpenCL device {}", device.name());
    auto source = cv::ocl::ProgramSource("", "message_passing_source", mKernelSource, "");

    cv::String errMsg;
    auto program = cv::ocl::Program(source, "", errMsg);
    if (program.ptr() == nullptr)
    {
        spdlog::error("Could not compile OpenCL programm {}", errMsg);
    }

    mKernel = cv::ocl::Kernel("message_passing", program);
}

Detections Tracker::trackContinous(const Unaries& unaries, const ManualUnaries& manualUnaries,
    const Settings& settings, const PairwiseTrafos& trafos)
{
    auto pairwiseKernel
        = getPairwiseKernel(settings.pairwiseSize, settings.pairwiseSigma).getUMat(cv::ACCESS_RW);
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
        pairwiseKernel, unaries.getUnaryDirectory());
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    spdlog::info("Elapsed time for tracking: {}", elapsed.count());

    return extractFromStates(states, ids, 0, settings, trafos);
}

void TrackerOpenCL::operator()(const cv::UMat& previousMessageToFactor,
    const cv::UMat& logPairwisePotential, cv::UMat& messageToNode, cv::Mat& phi)
{
    int pairwiseSize = logPairwisePotential.rows;
    int offset = std::floor(pairwiseSize / 2);

    auto rows = previousMessageToFactor.rows;
    auto cols = previousMessageToFactor.cols;
    if (mIds.empty())
    {
        const int size[] = {rows, cols, 2};
        mIds = cv::UMat(3, size, CV_32S, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    }

    std::size_t globalSize[2] = {static_cast<std::size_t>(rows), static_cast<std::size_t>(cols)};
    std::size_t localSize[2] = {32, 32};
    bool res = mKernel
                   .args(cv::ocl::KernelArg::PtrReadOnly(previousMessageToFactor),
                       cv::ocl::KernelArg::PtrReadOnly(logPairwisePotential),
                       cv::ocl::KernelArg::PtrWriteOnly(messageToNode),
                       cv::ocl::KernelArg::PtrWriteOnly(mIds), offset, rows, cols)
                   .run(2, globalSize, localSize, true);

    if (!res)
        spdlog::warn("OpenCL Kernel launch failed");

    mIds.copyTo(phi);
}

namespace
{
    cv::Mat truncatedMaxSum(std::size_t start, std::size_t end, const std::vector<std::size_t>& ids,
        const Unaries& unaries, const ManualUnaries& manualUnaries,
        const Tracker::Settings& settings, const cv::UMat& pairwiseKernel,
        const std::filesystem::path& workingDir)
    {
        auto openclTracker = TrackerOpenCL();
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
        cv::UMat firstUnary = unaries.at(ids[start]).getUMat(cv::ACCESS_RW);

        cv::UMat messageToNode = firstUnary.clone();
        cv::log(messageToNode, messageToNode);
        cv::UMat messageToFactor = messageToNode.clone();

        // backtracking data
        const int phiSize[] = {firstUnary.rows, firstUnary.cols, 2};

        if (numVariables == 1)
            messageToNode = messageToFactor;

        for (std::size_t i = start; i < end; i++)
        {
            std::size_t idx = ids[i];
            spdlog::debug("Optimize unary {} ({})", i, ids[i]);

            cv::Mat phi(3, phiSize, CV_32S);
            // set message to node by maximizing over log f and message to factor
            openclTracker(messageToFactor, pairwiseKernel, messageToNode, phi);
            Tracker::savePhi(i, phi, workingDir);

            cv::UMat currentUnary;
            if (manualUnaries.exists(idx))
                currentUnary = manualUnaries.unaryAt(idx).getUMat(cv::ACCESS_RW);
            else
                currentUnary = unaries.at(idx).getUMat(cv::ACCESS_RW);
            ;

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
                cv::add(currentUnary, cv::abs(tempMin), currentUnary);
                double multiplier = settings.manualMultiplier;
                cv::multiply(currentUnary, multiplier, currentUnary);
                cv::subtract(currentUnary, cv::abs(tempMin), currentUnary);
            }

            // pass message to factor by simple summation
            cv::add(messageToNode, currentUnary, messageToFactor);
        }

        // TODO: clean up this redundant mess here
        // find the max of the last message
        float logBestConfigurationProbability = std::numeric_limits<float>::lowest();
        float curVal = std::numeric_limits<float>::lowest();
        int curMaxI = 0;
        int curMaxJ = 0;
        cv::Mat messageToFactorCPU;
        messageToFactor.copyTo(messageToFactorCPU);

        for (int i = 0; i < messageToFactor.rows; i++)
        {
            for (int j = 0; j < messageToFactor.cols; j++)
            {
                curVal = messageToFactorCPU.at<float>(i, j);
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
            auto phi = Tracker::loadPhi(v, workingDir);
            maxStates.at<int>(v, 0) = phi.at<int>(r, c, 0);
            maxStates.at<int>(v, 1) = phi.at<int>(r, c, 1);
        }

        if (!workingDir.empty())
            fs::remove_all(workingDir / "phis");
        return maxStates;
    }
}
} // namespace ht
