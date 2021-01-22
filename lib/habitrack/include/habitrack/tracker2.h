#ifndef HABITRACK_TRACKER2_H
#define HABITRACK_TRACKER2_H

#include "habitrack/detections.h"
#include <opencv2/core.hpp>

namespace ht
{
class Unaries;
class ManualUnaries;
} // namespace ht

// TODO: replace with namespace and detail
namespace ht
{
class Tracker2
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
        const Settings& settings, std::size_t chunk, const matches::PairwiseTrafos& trafos);
    static Detections track(const Unaries& unaries, const ManualUnaries& manualUnaries,
        const Settings& settings, const matches::PairwiseTrafos& trafos);
    static std::size_t getNumChunks(std::size_t numUnaries, std::size_t chunkSize);
    static std::size_t getChunkEnd(
        std::size_t chunk, std::size_t numChunks, std::size_t chunkSize, std::size_t numUnaries);
};
} // namespace habitrack
#endif // HABITRACK_TRACKER2_H
