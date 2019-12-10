#ifndef HABITRACK_MATCHES_CONTAINER_H
#define HABITRACK_MATCHES_CONTAINER_H

#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

#include "habitrack/computeBehavior.h"
#include "habitrack/geometricType.h"
#include "habitrack/imageContainer.h"
#include "habitrack/isometry.h"
#include "habitrack/pairHash.h"
#include "habitrack/pairRecommender.h"

namespace ht
{
class BaseFeatureContainer;
}

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

class MatchesContainer : public std::enable_shared_from_this<MatchesContainer>
{
public:
    MatchesContainer(std::shared_ptr<BaseFeatureContainer> featureContainer,
        const std::filesystem::path& matchDir, MatchType matchType, std::size_t window,
        GeometricType geomType, double minCoverage = 0.0,
        std::unique_ptr<PairRecommender> recommender = nullptr);

    // for "automatic" matching
    void compute(std::size_t cacheSize, ComputeBehavior behavior = ComputeBehavior::Keep,
        const ImgIds& ids = ImgIds());

    // for "manual" matching, e.g. in KFS
    std::pair<Trafos, std::vector<Matches>> computePair(std::size_t idxI, std::size_t idxJ) const;

    PairwiseMatches getMatches(GeometricType geomType);
    PairwiseTrafos getTrafos(GeometricType geomType);

    GeometricType getUsableTypes(const ImgIds& ids = ImgIds());
    std::filesystem::path getMatchDir() const;

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
    std::vector<GeometricType> getTypeList() const;
    GeometricType getTypeFromFile(const std::filesystem::path& file) const;
    std::filesystem::path getFileName(detail::MatchTrafo matchTrafo, GeometricType geomType) const;
    bool checkIfExists(GeometricType geomType) const;

    PairwiseMatches getPutativeMatches(std::size_t cacheSize, const ImgIds& ids);
    PairwiseMatches getGeomMatches(
        std::size_t cacheSize, GeometricType type, PairwiseMatches&& matches);

    // helper function for different pair types
    std::vector<std::pair<std::size_t, std::size_t>> getPairList(
        std::size_t size, const ImgIds& ids) const;
    // TODO: outsource with strategy pattern --> PairSuggestor.getPairList();
    std::vector<std::pair<std::size_t, std::size_t>> getWindowPairList(
        std::size_t size, const ImgIds& ids) const;
    std::vector<std::pair<std::size_t, std::size_t>> getExhaustivePairList(
        std::size_t size, const ImgIds& ids) const;
    /* std::vector<std::pair<std::size_t, std::size_t>> getMILDPairList( */
    /*     std::size_t size, const ImgIds& ids) const; */

    cv::Ptr<cv::DescriptorMatcher> getMatcher() const;
    Matches putMatchPair(cv::Ptr<cv::DescriptorMatcher> descMatcher, const cv::Mat& descI,
        const cv::Mat& descJ) const;
    Matches putMatchPairHelper(cv::Ptr<cv::DescriptorMatcher> descMatcher, const cv::Mat& descI,
        const cv::Mat& descJ) const;
    std::pair<Trafo, Matches> geomMatchPair(const std::vector<cv::KeyPoint>& featI,
        const std::vector<cv::KeyPoint>& featJ, GeometricType filterType,
        const Matches& matches) const;

    // transformation fitting
    std::pair<std::vector<uchar>, cv::Mat> getInlierMask(const std::vector<cv::Point2f>& src,
        const std::vector<cv::Point2f>& dst, GeometricType type) const;
    std::pair<std::vector<uchar>, cv::Mat> getInlierMaskIsometry(
        const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst) const;
    std::pair<std::vector<uchar>, cv::Mat> getInlierMaskSimilarity(
        const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst) const;
    std::pair<std::vector<uchar>, cv::Mat> getInlierMaskAffinity(
        const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst) const;
    std::pair<std::vector<uchar>, cv::Mat> getInlierMaskHomography(
        const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst) const;

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

    void writeMatches(const PairwiseMatches& matches, GeometricType type) const;
    void writeTrafos(const PairwiseTrafos& matches, GeometricType type) const;
    PairwiseMatches loadMatches(GeometricType type) const;
    PairwiseTrafos loadTrafos(GeometricType type) const;

    inline size_t getInlierCount(const std::vector<uchar>& mask) const
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
    std::shared_ptr<BaseFeatureContainer> mFtContainer;
    std::filesystem::path mMatchDir;
    MatchType mMatchType;
    std::size_t mWindow;
    GeometricType mGeomType;
    double mMinCoverage;
    std::unique_ptr<PairRecommender> mRecommender;
    bool mIsComputed;
};
} // namespace ht

#endif // HABITRACK_Matches_CONTAINER_H
