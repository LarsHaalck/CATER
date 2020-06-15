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
    struct Settings
    {
        double subsample;
        int pairwiseSize;
        double pairwiseSigma;
        double manualMultiplier;
        bool calculateBearing;
        int windowSize;
        int outlierTolerance;
    };

public:
    static Detections track(const Unaries& unaries, const ManualUnaries& manualUnaries,
        const Settings& settings, std::size_t chunk, std::size_t chunkSize);
    static Detections track(const Unaries& unaries, const ManualUnaries& manualUnaries,
        const Settings& settings, std::size_t chunkSize = 0);
    static std::size_t getNumChunks(std::size_t numUnaries, std::size_t chunkSize);
    static std::size_t getChunkEnd(
        std::size_t chunk, std::size_t numChunks, std::size_t chunkSize, std::size_t numUnaries);

private:
    static cv::Mat getPairwiseKernel(int size, double sigma);
    static cv::Mat truncatedMaxSum(std::size_t start, std::size_t end,
        const std::vector<std::size_t>& ids, const Unaries& unaries,
        const ManualUnaries& manualUnaries, const Settings& settings,
        const cv::Mat& pairwiseKernel);

    static void passMessageToNode(const cv::Mat& previousMessageToFactor,
        const cv::Mat& logPairwisePotential, cv::Mat& messageToNode, cv::Mat& phi);
    static void passMessageToFactor(const cv::Mat& previousMessageToNode,
        const cv::Mat& unaryPotential, cv::Mat& messageToFactor);

    static Detections extractFromStates(const cv::Mat& states, const std::vector<std::size_t>& ids,
        std::size_t offset, double subsample);
};
} // namespace habitrack
#endif // HABITRACK_TRACKER_H
