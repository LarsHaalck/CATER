#ifndef TRACKER_TRACKEROPENCL_H
#define TRACKER_TRACKEROPENCL_H

#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>

namespace ht
{
class TrackerOpenCL
{
public:
    TrackerOpenCL();
    void operator()(const cv::UMat& previousMessageToFactor, const cv::UMat& logPairwisePotential,
        cv::UMat& messageToNode, cv::Mat& phi);

private:
    void initMemory(int rows, int cols, int type);

private:
    static const char* mKernelSource;
    cv::ocl::Kernel mKernel;
    cv::UMat mIds;
};
} // namespace ht

#endif // TRACKER_TRACKEROPENCL_H
