#ifndef HABITRACK_MATCHES_CONTAINER_H
#define HABITRACK_MATCHES_CONTAINER_H

#include <filesystem>
#include <vector>

#include "habitrack/computeBehavior.h"
#include "habitrack/imageType.h"
#include "habitrack/geometricType.h"


namespace ht
{
    class FeatureContainer;
}

namespace ht
{
enum class MatchType
{
    Exhaustive,
    MILD,
    /* NN, */
    Windowed
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

    /* std::vector<cv::KeyPoint> featureAt(std::size_t idx, ImageType imageType); */
    /* cv::Mat descriptorAt(std::size_t idx, ImageType imageType); */

    /* std::unique_ptr<FeatureCache> getFeatureCache(std::size_t maxChunkSize, */
    /*     ImageType imageType); */
    /* std::unique_ptr<DescriptorCache> getDescriptorCache(std::size_t maxChunkSize, */
    /*     ImageType imageType); */
private:
    GeometricType getTypeFromFile(const std::filesystem::path& file);
    std::filesystem::path getFileName(detail::MatchTrafo matchTrafo,
        GeometricType geomType);
    bool checkIfExists(GeometricType geomType);

    void getPutativeMatches(std::size_t cacheSize, ComputeBehavior behavior);
    std::vector<std::pair<std::size_t, std::size_t>> getPairList(std::size_t size);
    std::vector<std::pair<std::size_t, std::size_t>> getWindowPairList(std::size_t size);
    std::vector<std::pair<std::size_t, std::size_t>> getMILDPairList(std::size_t size);
    std::vector<std::pair<std::size_t, std::size_t>> getExhaustivePairList(
        std::size_t size);

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
