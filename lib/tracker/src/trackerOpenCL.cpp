#include "tracker/tracker.h"

#include "image-processing/transformation.h"
#include "image-processing/util.h"
#include "spdlog/spdlog.h"
#include "tracker/manualUnaries.h"
#include "tracker/trackerOpenCL.h"
#include "tracker/unaries.h"
#include "util/algorithm.h"
#include <chrono>
#include <iostream>
#include <limits>
#include <opencv2/imgproc.hpp>

namespace ht
{
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
    auto openclTracker = TrackerOpenCL();
    auto states = truncatedMaxSum(
        0, numUnaries, ids, unaries, manualUnaries, settings, pairwiseKernel, openclTracker);
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    spdlog::info("Elapsed time for tracking: {}", elapsed.count());

    return extractFromStates(states, ids, 0, settings, trafos);
}

void TrackerOpenCL::initMemory(int rows, int cols, int type)
{
    mPrev = cv::UMat(rows, cols, type, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    mCurr = cv::UMat(rows, cols, type, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    const int size[] = {rows, cols, 2};
    mIds = cv::UMat(3, size, CV_32S, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
}

void TrackerOpenCL::operator()(const cv::Mat& previousMessageToFactor,
    const cv::Mat& logPairwisePotential, cv::Mat& messageToNode, cv::Mat& phi)
{
    int pairwiseSize = logPairwisePotential.rows;
    int offset = std::floor(pairwiseSize / 2);

    auto rows = previousMessageToFactor.rows;
    auto cols = previousMessageToFactor.cols;
    if (mPair.empty())
    {
        initMemory(rows, cols, previousMessageToFactor.type());
        // mPair mPair will never change
        mPair = previousMessageToFactor.getUMat(cv::ACCESS_READ);
    }
    // copy last message, rest will be overwritten
    previousMessageToFactor.copyTo(mPrev);

    std::size_t globalSize[2] = {static_cast<std::size_t>(rows), static_cast<std::size_t>(cols)};
    std::size_t localSize[2] = {32, 32};
    bool res
        = mKernel
              .args(cv::ocl::KernelArg::PtrReadOnly(mPrev),
                  cv::ocl::KernelArg::PtrReadOnly(mPair),
                  cv::ocl::KernelArg::PtrWriteOnly(mCurr),
                  cv::ocl::KernelArg::PtrWriteOnly(mIds), offset, rows, cols)
              .run(2, globalSize, localSize, true);

    if (!res)
        spdlog::warn("OpenCL Kernel launch failed");

    mIds.copyTo(phi);
    mCurr.copyTo(messageToNode);
}
} // namespace ht
