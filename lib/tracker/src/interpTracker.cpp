#include <habitrack/tracker/interpTracker.h>

#include <habitrack/image-processing/transformation.h>
#include <habitrack/image-processing/util.h>
#include <habitrack/tracker/manualUnaries.h>
#include <habitrack/tracker/unaries.h>
#include <habitrack/util/algorithm.h>
#include <habitrack/util/threadPool.h>

#include <spdlog/spdlog.h>
#include <chrono>
#include <future>
#include <iostream>
#include <limits>
#include <opencv2/imgproc.hpp>

namespace ht
{
using namespace matches;
using namespace transformation;

Detections InterpTracker::track(const Unaries& unaries, const ManualUnaries& manualUnaries,
    const Settings& settings, std::size_t, const PairwiseTrafos& trafos)
{
    return InterpTracker::track(unaries, manualUnaries, settings, trafos);
}

Detections InterpTracker::track(const Unaries& unaries, const ManualUnaries& manualUnaries,
    const Settings& settings, const PairwiseTrafos&)
{
    spdlog::warn("Interpolation Tracker running");
    auto ids = unaries.getIDs();
    auto manual_ids = size_t_vec();

    auto unSize = unaries.at(ids[0]).size();
    auto upsampling = 1.0 / settings.subsample;
    auto center = upsampling * cv::Point(unSize.width, unSize.height) / 2;

    for (auto id : ids)
    {
        if (manualUnaries.exists(id))
            manual_ids.push_back(id);
    }

    // should not be needed
    std::sort(std::begin(manual_ids), std::end(manual_ids));

    Detections detections;

    // from first unary id to first manual unary id use center of image
    for (std::size_t i = ids[0]; i < manual_ids[0]; i++)
        detections.insert(i, {center, 0, 0});

    // rest interpolate
    for (std::size_t i = 0; i < manual_ids.size() - 1; i++)
    {
        auto curr = manual_ids[i];
        auto next = manual_ids[i + 1];
        auto curr_pt = manualUnaries.unaryPointAt(curr);
        auto next_pt = manualUnaries.unaryPointAt(next);

        for (std::size_t j = curr; j < next; j++)
        {
            // 0 when curr, 1 when next
            double alpha = static_cast<double>(j - curr) / (next - curr);
            auto pt = alpha * next_pt + (1 - alpha) * curr_pt;
            if (pt.x < 0 || pt.y < 0)
                spdlog::critical("Negative pt ({},{}) interpolate from ({},{}) ->  ({},{}) with a: "
                                 "{} from {} and {}",
                    pt.x, pt.y, curr_pt.x, curr_pt.y, next_pt.x, next_pt.y, alpha, curr, next);
            detections.insert(j, {pt, 0, 0});
        }
    }

    // from last manual unary id to last unary id use center of image
    for (std::size_t i = manual_ids[manual_ids.size() - 1]; i < ids[ids.size() - 1]; i++)
        detections.insert(i, {center, 0, 0});

    return detections;
}

std::size_t InterpTracker::getNumChunks(std::size_t numUnaries, std::size_t chunkSize)
{
    if (chunkSize == 0)
        chunkSize = numUnaries;

    auto numChunks = std::max(numUnaries / chunkSize, static_cast<std::size_t>(1));
    return numChunks;
}

std::size_t InterpTracker::getChunkEnd(
    std::size_t chunk, std::size_t numChunks, std::size_t chunkSize, std::size_t numUnaries)
{
    if (chunk == numChunks - 1)
        return numUnaries;
    return std::min(numUnaries, (chunk + 1) * chunkSize);
}
} // namespace ht
