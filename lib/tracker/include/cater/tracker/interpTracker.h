#ifndef TRACKER_INTERPTRACKER_H
#define TRACKER_INTERPTRACKER_H

#include <cater/tracker/detections.h>
#include <cater/tracker/tracker.h>
#include <opencv2/core.hpp>

namespace ct
{
class Unaries;
class ManualUnaries;
} // namespace ct

// TODO: replace with namespace and detail
namespace ct
{
class InterpTracker
{
public:
public:
    static Detections track(const Unaries& unaries, const ManualUnaries& manualUnaries,
        const ct::Tracker::Settings& settings, std::size_t chunk, const PairwiseTrafos& trafos);
    static Detections track(const Unaries& unaries, const ManualUnaries& manualUnaries,
        const ct::Tracker::Settings& settings, const PairwiseTrafos& trafos);
};
} // namespace ct
#endif // TRACKER_INTERPTRACKER_H
