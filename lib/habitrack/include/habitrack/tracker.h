#ifndef HABITRACK_TRACKER_H
#define HABITRACK_TRACKER_H

#include <opencv2/core.hpp>
#include "habitrack/detections.h"

namespace ht
{
class Unaries;
class ManualUnaries;
} // namespace ht

namespace ht
{
class Tracker
{
public:
    struct UnarySettings
    {
        double subsample;
        int pairwiseSize;
        double pairwiseSigma;
        double manualMultiplier;
    };

    struct SmoothBearingSettings
    {
        bool calculate;
        int windowSize;
        int outlierTolerance;
    };

public:
    static Detections track(const Unaries& unaries, const ManualUnaries& manualUnaries,
        UnarySettings unarySettings, SmoothBearingSettings smoothBearingSettings,
        std::size_t chunkSize = 0);

private:
    static cv::Mat getPairwiseKernel(int size, double sigma);
    static cv::Mat truncatedMaxSum(std::size_t start, std::size_t end,
        const std::vector<std::size_t>& ids, const Unaries& unaries,
        const ManualUnaries& manualUnaries, UnarySettings unarySettings,
        const cv::Mat& pairwiseKernel);

    static void passMessageToNode(const cv::Mat& previousMessageToFactor,
        const cv::Mat& logPairwisePotential, cv::Mat& messageToNode, cv::Mat& phi);
    static void passMessageToFactor(const cv::Mat& previousMessageToNode,
        const cv::Mat& unaryPotential, cv::Mat& messageToFactor);
};
} // namespace habitrack
#endif // HABITRACK_TRACKER_H
