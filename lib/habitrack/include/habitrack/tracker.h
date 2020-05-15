#ifndef HABITRACK_TRACKER_H
#define HABITRACK_TRACKER_H

#include <opencv2/core.hpp>

namespace ht
{
class Unaries;
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
    static void track(const Unaries& unaries, UnarySettings unarySettings,
        SmoothBearingSettings smoothBearingSettings);
    static cv::Mat get2DGaussianKernel(int size, double sigma);
};
} // namespace habitrack
#endif // HABITRACK_TRACKER_H
