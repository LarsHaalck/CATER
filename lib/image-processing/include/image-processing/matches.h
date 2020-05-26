#ifndef HABITRACK_MATCHES_CONTAINER_H
#define HABITRACK_MATCHES_CONTAINER_H

#include "image-processing/baseFeatureContainer.h"
#include "image-processing/geometricType.h"
#include "image-processing/pairHash.h"
#include "image-processing/pairRecommender.h"
#include "progressbar/baseProgressBar.h"
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

namespace ht::matches
{
using Trafo = cv::Mat;
using Trafos = std::vector<cv::Mat>;
using Match = cv::DMatch;
using Matches = std::vector<cv::DMatch>;

using PairwiseMatches = std::unordered_map<std::pair<std::size_t, std::size_t>, Matches>;
using PairwiseTrafos = std::unordered_map<std::pair<std::size_t, std::size_t>, Trafo>;

enum class MatchType
{
    Exhaustive,
    Windowed,
    Manual,
    Strategy
};

namespace detail
{
    enum class MatchTrafo
    {
        Match,
        Trafo
    };
} // namespace detail

bool isComputed(const std::filesystem::path& matchDir, GeometricType geomType);

// for "automatic" matching
bool compute(const std::filesystem::path& matchDir, GeometricType geomType,
    const BaseFeatureContainer& features, MatchType matchType, std::size_t window = 0,
    double minCoverage = 0.0, std::unique_ptr<PairRecommender> recommender = nullptr,
    std::size_t cacheSize = 0, const size_t_vec& ids = size_t_vec(),
    std::shared_ptr<BaseProgressBar> cb = {});

// for "manual" matching, e.g. in KFS
std::pair<Trafos, std::vector<Matches>> computePair(GeometricType geomType,
    const BaseFeatureContainer& features, std::size_t idxI, std::size_t idxJ,
    double coverage = 0.0);

template <typename T>
static std::vector<std::pair<std::size_t, std::size_t>> getKeyList(
    const std::unordered_map<std::pair<std::size_t, std::size_t>, T>& map)
{
    auto keys = std::vector<std::pair<std::size_t, std::size_t>>();
    keys.reserve(map.size());
    for (const auto& match : map)
        keys.push_back(match.first);

    std::sort(std::begin(keys), std::end(keys));
    return keys;
}

GeometricType getConnectedTypes(
    const std::filesystem::path& matchDir, GeometricType geomType, size_t numImgs);
GeometricType getConnectedTypes(
    const std::filesystem::path& matchDir, GeometricType geomType, const size_t_vec& ids);

PairwiseMatches getMatches(const std::filesystem::path& matchDir, GeometricType geomType);
PairwiseTrafos getTrafos(const std::filesystem::path& matchDir, GeometricType geomType);

namespace detail
{
    std::vector<GeometricType> getTypeList(GeometricType type);
    std::filesystem::path getFileName(const std::filesystem::path& matchDir,
        detail::MatchTrafo matchTrafo, GeometricType geomType);
    bool checkIfExists(const std::filesystem::path& matchDir, GeometricType geomType);

    PairwiseMatches getPutativeMatches(const std::filesystem::path& matchDir,
        const BaseFeatureContainer& fts, MatchType matchType, std::size_t window,
        std::unique_ptr<PairRecommender> recommender, std::size_t cacheSize, const size_t_vec& ids,
        std::shared_ptr<BaseProgressBar> cb);

    PairwiseMatches getGeomMatches(const std::filesystem::path& matchDir,
        const BaseFeatureContainer& fts, GeometricType geomType, double minCoverage, int area,
        std::size_t cacheSize, PairwiseMatches&& matches, std::shared_ptr<BaseProgressBar> cb);

        // helper function for different pair types
        std::vector<std::pair<std::size_t, std::size_t>> getPairList(MatchType type,
            std::size_t size, std::size_t window, std::unique_ptr<PairRecommender> recommender,
            const size_t_vec& ids);
    std::vector<std::pair<std::size_t, std::size_t>> getWindowPairList(
        std::size_t size, std::size_t window, const size_t_vec& ids);
    std::vector<std::pair<std::size_t, std::size_t>> getExhaustivePairList(
        std::size_t size, const size_t_vec& ids);

    cv::Ptr<cv::DescriptorMatcher> getMatcher(FeatureType featureType);
    Matches putMatchPair(
        cv::Ptr<cv::DescriptorMatcher> descMatcher, const cv::Mat& descI, const cv::Mat& descJ);
    Matches putMatchPairHelper(
        cv::Ptr<cv::DescriptorMatcher> descMatcher, const cv::Mat& descI, const cv::Mat& descJ);
    std::pair<Trafo, Matches> geomMatchPair(const std::vector<cv::KeyPoint>& featI,
        const std::vector<cv::KeyPoint>& featJ, GeometricType filterType, double minCoverage,
        int Area, const Matches& matches);

    // transformation fitting
    std::pair<std::vector<uchar>, cv::Mat> getInlierMask(const std::vector<cv::Point2f>& src,
        const std::vector<cv::Point2f>& dst, GeometricType type);
    std::pair<std::vector<uchar>, cv::Mat> getInlierMaskIsometry(
        const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst);
    std::pair<std::vector<uchar>, cv::Mat> getInlierMaskSimilarity(
        const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst);
    std::pair<std::vector<uchar>, cv::Mat> getInlierMaskAffinity(
        const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst);
    std::pair<std::vector<uchar>, cv::Mat> getInlierMaskHomography(
        const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst);

    std::string typeToString(GeometricType type);

    template <typename T>
    void filterEmptyPairwise(T& matches)
    {
        for (auto it = std::begin(matches); it != std::end(matches);)
        {
            if (it->second.empty())
                it = matches.erase(it);
            else
                ++it;
        }
    }

    void writeMatches(
        const std::filesystem::path& machDir, const PairwiseMatches& matches, GeometricType type);
    void writeTrafos(
        const std::filesystem::path& machDir, const PairwiseTrafos& matches, GeometricType type);

    PairwiseMatches loadMatches(const std::filesystem::path& matchDir, GeometricType type);
    PairwiseTrafos loadTrafos(const std::filesystem::path& matchDir, GeometricType type);

    inline size_t getInlierCount(const std::vector<uchar>& mask)
    {
        size_t count = 0;
        for (const auto id : mask)
        {
            if (id)
                count++;
        }
        return count;
    }
} // namespace detail

} // namespace ht

#endif // HABITRACK_Matches_CONTAINER_H
