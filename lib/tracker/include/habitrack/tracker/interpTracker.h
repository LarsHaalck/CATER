#ifndef TRACKER_INTERPTRACKER_H
#define TRACKER_INTERPTRACKER_H

#include <habitrack/tracker/detections.h>
#include <opencv2/core.hpp>

namespace ht
{
class Unaries;
class ManualUnaries;
} // namespace ht

// TODO: replace with namespace and detail
namespace ht
{
class InterpTracker
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
        const Settings& settings, const PairwiseTrafos& trafos);
    static std::size_t getNumChunks(std::size_t numUnaries, std::size_t chunkSize);
    static std::size_t getChunkEnd(
        std::size_t chunk, std::size_t numChunks, std::size_t chunkSize, std::size_t numUnaries);
};
} // namespace ht
#endif // TRACKER_INTERPTRACKER_H
