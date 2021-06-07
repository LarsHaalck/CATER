#ifndef HABITACK_KEY_FRAME_SELECTOR_H
#define HABITACK_KEY_FRAME_SELECTOR_H

#include <algorithm>
#include <filesystem>
#include <memory>
#include <opencv2/features2d.hpp>
#include <vector>

#include "image-processing/baseFeatureContainer.h"
#include "image-processing/matches.h"
#include "progressbar/progressBar.h"

namespace ht::KeyFrames
{
namespace detail
{
    std::pair<float, float> getRealLowHigh(cv::Size imgSize, float low, float high);
    std::pair<float, std::size_t> getMedianDistanceShift(
        std::size_t idI, std::size_t idJ, const BaseFeatureContainer& fts, GeometricType type);

    std::size_t filterViews(
        std::vector<std::pair<float, std::size_t>> distOverlapVec, float low, float high);
    bool compareMaxOverlap(const std::pair<float, std::size_t>& lhs,
        const std::pair<float, std::size_t>& rhs, float low, float high);
    double calcReprojError(const std::vector<cv::Point2f>& ptsSrc, std::vector<cv::Point2f>& ptsDst,
        const cv::Mat& trafo);

    void writeToFile(const std::filesystem::path& file, const std::vector<std::size_t>& keyFrames);
    std::vector<std::size_t> loadFromFile(const std::filesystem::path& file);
}

bool isComputed(const std::filesystem::path& file);
std::vector<std::size_t> fromDir(const std::filesystem::path& file);

std::vector<std::size_t> compute(const BaseFeatureContainer& ftContainer, GeometricType type,
    const std::filesystem::path& file, float low, float high,
    std::shared_ptr<BaseProgressBar> cb = {});

} // namespace ht
#endif // HABITACK_KEY_FRAME_SELECTOR_H
