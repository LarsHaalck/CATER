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

#include "matchesIO.h"
#include "pairHash.h"

/* #include "habitrack/matchesCache.h" */
/* #include "habitrack/trafoCache.h" */

namespace ht
{
class FeatureContainer;
}

namespace ht
{
using Trafo = cv::Mat;
using Trafos = std::vector<cv::Mat>;
using Match = cv::DMatch;
using Matches = std::vector<cv::DMatch>;
using PairWiseMatches = std::unordered_map<std::pair<std::size_t, std::size_t>, Matches>;

enum class MatchType
{
    Exhaustive,
    MILD,
    /* NN, */
    Windowed,
    Manual
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
    MatchesContainer(std::shared_ptr<FeatureContainer> featureContainer,
        const std::filesystem::path& matchDir, MatchType matchType, std::size_t window,
        GeometricType geomType);

    void compute(std::size_t cacheSize, ComputeBehavior behavior = ComputeBehavior::Keep);

    // for manual matching
    std::pair<Trafos, std::vector<Matches>> compute(std::size_t idxI, std::size_t idxJ);

    /* std::vector<cv::DMatch> matchesAt(ImgId idI, ImgId idj); */
    /* cv::Mat trafoAt(ImgId idI, ImgId idj, GeometricType geomType); */

    /* std::unique_ptr<MatchesCache> getMatchesCache(std::size_t maxChunkSize, */
    /*     const ImgIds& ids = ImgIds()); */
    /* std::unique_ptr<TrafoCache> getTrafoCache(std::size_t maxChunkSize, */
    /*     const ImgIds& ids = ImgIds(); */
private:
    GeometricType getTypeFromFile(const std::filesystem::path& file) const;
    std::filesystem::path getFileName(detail::MatchTrafo matchTrafo, GeometricType geomType) const;
    bool checkIfExists(GeometricType geomType) const;

    PairWiseMatches getPutativeMatches(std::size_t cacheSize);
    PairWiseMatches getGeomMatches(
        std::size_t cacheSize, GeometricType type, PairWiseMatches&& matches);
    std::vector<std::pair<std::size_t, std::size_t>> getPairList(std::size_t size) const;
    std::vector<std::pair<std::size_t, std::size_t>> getWindowPairList(std::size_t size) const;
    std::vector<std::pair<std::size_t, std::size_t>> getExhaustivePairList(std::size_t size) const;

    std::vector<std::pair<std::size_t, std::size_t>> getPairList(
        const PairWiseMatches& matches) const;

    cv::Ptr<cv::DescriptorMatcher> getMatcher() const;
    Matches putMatch(
        cv::Ptr<cv::DescriptorMatcher> descMatcher, const cv::Mat& descI, const cv::Mat& descJ);
    std::pair<Trafo, Matches> geomMatch(const std::vector<cv::KeyPoint>& featI,
        const std::vector<cv::KeyPoint>& featJ, GeometricType filterType, const Matches& matches);
    GeometricType findNextBestModel(GeometricType currType);

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
    void filterEmptyMatches(PairWiseMatches& matches);

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

    void writeMatches(const PairWiseMatches& matches, GeometricType type) const;
    /* cv::Ptr<cv::Feature2D> getFtPtr(); */
    /* void writeChunk(std::pair<std::size_t, std::size_t> bounds, */
    /*     const std::vector<std::vector<cv::KeyPoint>>& fts, */
    /*     const std::vector<cv::Mat>& descs); */
    /* void writeFts(const std::filesystem::path& file, */
    /*     const std::vector<cv::KeyPoint>& fts); */
    /* void writeDescs(const std::filesystem::path& file, const cv::Mat& descs); */
private:
    std::shared_ptr<FeatureContainer> mFtContainer;
    std::filesystem::path mMatchDir;
    MatchType mMatchType;
    std::size_t mWindow;
    GeometricType mGeomType;
    bool mIsComputed;
};
} // namespace ht

#endif // HABITRACK_Matches_CONTAINER_H
