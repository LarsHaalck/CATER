#ifndef HABITRACK_MATCHES_CONTAINER_H
#define HABITRACK_MATCHES_CONTAINER_H

#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

#include "habitrack/baseFeatureContainer.h"
#include "habitrack/geometricType.h"
#include "habitrack/images.h"
#include "habitrack/isometry.h"
#include "habitrack/pairHash.h"
#include "habitrack/pairRecommender.h"

namespace ht
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

class MatchesContainer
{
public:
    MatchesContainer() = default;

    static bool isComputed(const std::filesystem::path& matchDir, GeometricType geomType);

    static MatchesContainer compute(const std::filesystem::path& matchDir, GeometricType geomType,
        const BaseFeatureContainer& featureContainer, MatchType matchType, std::size_t window = 0,
        double minCoverage = 0.0, std::unique_ptr<PairRecommender> recommender = nullptr,
        std::size_t cacheSize = 0, const size_t_vec& ids = size_t_vec());

    /* // for "automatic" matching */
    /* void compute(std::size_t cacheSize, ComputeBehavior behavior = ComputeBehavior::Keep, */
    /*     const size_t_vec& ids = size_t_vec()); */

    /* // for "manual" matching, e.g. in KFS */
    /* std::pair<Trafos, std::vector<Matches>> computePair(std::size_t idxI, std::size_t idxJ) const; */

    /* PairwiseMatches getMatches(GeometricType geomType); */
    /* PairwiseTrafos getTrafos(GeometricType geomType); */

    /* GeometricType getUsableTypes(const size_t_vec& ids = size_t_vec()); */
    /* std::filesystem::path getMatchDir() const; */

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

private:
    static std::vector<GeometricType> getTypeList(GeometricType type);
    static std::filesystem::path getFileName(const std::filesystem::path& matchDir,
            detail::MatchTrafo matchTrafo, GeometricType geomType);
    static bool checkIfExists(const std::filesystem::path& matchDir, GeometricType geomType);

    static PairwiseMatches getPutativeMatches(const std::filesystem::path& matchDir,
    const BaseFeatureContainer& fts, MatchType matchType, std::size_t window,
    std::unique_ptr<PairRecommender> recommender, std::size_t cacheSize, const size_t_vec& ids);

    static PairwiseMatches getGeomMatches(const std::filesystem::path& matchDir,
        const BaseFeatureContainer& fts, GeometricType geomType, double minCoverage, int area,
        std::size_t cacheSize, PairwiseMatches&& matches);

    // helper function for different pair types
    static std::vector<std::pair<std::size_t, std::size_t>> getPairList(MatchType type,
        std::size_t size, std::size_t window, std::unique_ptr<PairRecommender> recommender,
        const size_t_vec& ids);
    static std::vector<std::pair<std::size_t, std::size_t>> getWindowPairList(
        std::size_t size, std::size_t window, const size_t_vec& ids);
    static std::vector<std::pair<std::size_t, std::size_t>> getExhaustivePairList(
        std::size_t size, const size_t_vec& ids);

    static cv::Ptr<cv::DescriptorMatcher> getMatcher(FeatureType featureType);
    static Matches putMatchPair(cv::Ptr<cv::DescriptorMatcher> descMatcher, const cv::Mat& descI,
        const cv::Mat& descJ);
    static Matches putMatchPairHelper(cv::Ptr<cv::DescriptorMatcher> descMatcher, const cv::Mat& descI,
        const cv::Mat& descJ);
    static std::pair<Trafo, Matches> geomMatchPair(const std::vector<cv::KeyPoint>& featI,
        const std::vector<cv::KeyPoint>& featJ, GeometricType filterType, double minCoverage,
        int Area, const Matches& matches);

    // transformation fitting
    static std::pair<std::vector<uchar>, cv::Mat> getInlierMask(const std::vector<cv::Point2f>& src,
        const std::vector<cv::Point2f>& dst, GeometricType type);
    static std::pair<std::vector<uchar>, cv::Mat> getInlierMaskIsometry(
        const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst);
    static std::pair<std::vector<uchar>, cv::Mat> getInlierMaskSimilarity(
        const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst);
    static std::pair<std::vector<uchar>, cv::Mat> getInlierMaskAffinity(
        const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst);
    static std::pair<std::vector<uchar>, cv::Mat> getInlierMaskHomography(
        const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst);

    static std::string typeToString(GeometricType type);

    template <typename T>
    static void filterEmptyPairwise(T& matches)
    {
        for (auto it = std::begin(matches); it != std::end(matches);)
        {
            if (it->second.empty())
                it = matches.erase(it);
            else
                ++it;
        }
    }

    static void writeMatches(const std::filesystem::path& machDir, const PairwiseMatches& matches,
        GeometricType type);
    static void writeTrafos(const std::filesystem::path& machDir, const PairwiseTrafos& matches,
        GeometricType type);

    /* PairwiseMatches loadMatches(GeometricType type) const; */
    /* PairwiseTrafos loadTrafos(GeometricType type) const; */

    inline static size_t getInlierCount(const std::vector<uchar>& mask)
    {
        size_t count = 0;
        for (const auto id : mask)
        {
            if (id)
                count++;
        }
        return count;
    }

private:
    /* std::shared_ptr<BaseFeatureContainer> mFtContainer; */
    /* std::filesystem::path mMatchDir; */
    /* MatchType mMatchType; */
    /* std::size_t mWindow; */
    /* GeometricType mGeomType; */
    /* double mMinCoverage; */
    /* std::unique_ptr<PairRecommender> mRecommender; */
    /* bool mIsComputed; */
};
} // namespace ht

#endif // HABITRACK_Matches_CONTAINER_H
