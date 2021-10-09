#ifndef TRACKER_TRACKER_H
#define TRACKER_TRACKER_H

#include "tracker/detections.h"
#include <opencv2/core.hpp>

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
        bool smoothBearing;
        int windowSize;
        int outlierTolerance;
        std::size_t chunkSize;
    };

public:
    static Detections track(const Unaries& unaries, const ManualUnaries& manualUnaries,
        const Settings& settings, std::size_t chunk, const PairwiseTrafos& trafos);

    static Detections track(const Unaries& unaries, const ManualUnaries& manualUnaries,
        const Settings& settings, const PairwiseTrafos& trafos)
    {
        auto chunkSize = settings.chunkSize;
        if (chunkSize > 0)
            return trackChunked(unaries, manualUnaries, settings, trafos);
        return trackContinous(unaries, manualUnaries, settings, trafos);
    }

    static Detections trackChunked(const Unaries& unaries, const ManualUnaries& manualUnaries,
        const Settings& settings, const PairwiseTrafos& trafos);
    static Detections trackContinous(const Unaries& unaries, const ManualUnaries& manualUnaries,
        const Settings& settings, const PairwiseTrafos& trafos);

private:
    static cv::Mat getPairwiseKernel(int size, double sigma);

    template <typename MessagePassing>
    static cv::Mat truncatedMaxSum(std::size_t start, std::size_t end,
        const std::vector<std::size_t>& ids, const Unaries& unaries,
        const ManualUnaries& manualUnaries, const Settings& settings, const cv::Mat& pairwiseKernel,
        MessagePassing messagePassing);

    static Detections extractFromStates(const cv::Mat& states, const std::vector<std::size_t>& ids,
        std::size_t offset, const Settings& settings, const PairwiseTrafos& trafos);

    static std::pair<double, double> calcBearingAndQuality(std::size_t lastIdx, std::size_t idx,
        cv::Point lastPos, cv::Point pos, const PairwiseTrafos& trafos);

    static void smoothBearing(Detections& detections, const Settings& settings);
    static std::vector<double> filterAndNormaliseLengthVec(
        const Detections& detections, int outlierTolerance);
    static std::vector<std::vector<double>> getWindows(
        const std::vector<double>& vec, std::size_t windowSize);
    static double calcWeightedCircularMean(
        const std::vector<double>& weightsWindow, const std::vector<double>& anglesWindow);
};
} // namespace ht
#endif // TRACKER_TRACKER_H
