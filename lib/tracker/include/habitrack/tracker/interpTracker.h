#ifndef TRACKER_INTERPTRACKER_H
#define TRACKER_INTERPTRACKER_H

#include <habitrack/tracker/detections.h>
#include <habitrack/tracker/tracker.h>
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
public:
    static Detections track(const Unaries& unaries, const ManualUnaries& manualUnaries,
        const ht::Tracker::Settings& settings, std::size_t chunk, const PairwiseTrafos& trafos);
    static Detections track(const Unaries& unaries, const ManualUnaries& manualUnaries,
        const ht::Tracker::Settings& settings, const PairwiseTrafos& trafos);
};
} // namespace ht
#endif // TRACKER_INTERPTRACKER_H
