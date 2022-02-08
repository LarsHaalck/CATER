#ifndef HABITACK_KEY_FRAME_SELECTOR_H
#define HABITACK_KEY_FRAME_SELECTOR_H

#include <algorithm>
#include <filesystem>
#include <memory>
#include <numeric>
#include <vector>

#include <opencv2/features2d.hpp>

#include <habitrack/image-processing/baseFeatureContainer.h>
#include <habitrack/image-processing/matches.h>
#include <habitrack/progressbar/progressBar.h>

namespace ht::KeyFrames
{
enum class Strategy
{
    Matches,
    Borda
};

namespace detail
{
    // shift, num, coverage, response, -reprojError
    using KeyFrameData = std::tuple<float, std::size_t, float, float, float>;

    std::pair<float, float> getRealLowHigh(cv::Size imgSize, float low, float high);
    KeyFrameData getMedianDistanceShift(
        std::size_t idI, std::size_t idJ, const BaseFeatureContainer& fts, GeometricType type);

    std::size_t filterViews(
        std::vector<KeyFrameData> distOverlapVec, float low, float high, Strategy strategy);
    bool compareMaxOverlap(const std::pair<float, std::size_t>& lhs,
        const std::pair<float, std::size_t>& rhs, float low, float high);
    double calcReprojError(const std::vector<cv::Point2f>& ptsSrc, std::vector<cv::Point2f>& ptsDst,
        const cv::Mat& trafo);

    void writeToFile(const std::filesystem::path& file, const std::vector<std::size_t>& keyFrames);
    std::vector<std::size_t> loadFromFile(const std::filesystem::path& file);

    template <std::size_t I>
    std::vector<std::size_t> getBordaCount(const std::vector<KeyFrameData>& data)
    {
        std::vector<std::size_t> ids(data.size());
        std::iota(std::begin(ids), std::end(ids), 0);
        std::stable_sort(std::begin(ids), std::end(ids),
            [&](auto idI, auto idJ) { return std::get<I>(data[idI]) < std::get<I>(data[idJ]); });
        std::transform(std::begin(ids), std::end(ids), std::begin(ids),
            [&](auto id) { return (std::get<I>(data[id]) == 0) ? 0 : id; });
        return ids;
    }

    std::vector<std::pair<float, std::size_t>> getBordaCounts(
        const std::vector<KeyFrameData>& data, Strategy strategy);
}

bool isComputed(const std::filesystem::path& file);
std::vector<std::size_t> fromDir(const std::filesystem::path& file);

std::vector<std::size_t> compute(const BaseFeatureContainer& ftContainer, GeometricType type,
    const std::filesystem::path& file, float low, float high, Strategy strategy,
    std::shared_ptr<BaseProgressBar> cb = {});

} // namespace ht
#endif // HABITACK_KEY_FRAME_SELECTOR_H
