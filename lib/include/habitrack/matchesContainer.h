#ifndef HABITRACK_MATCHES_CONTAINER_H
#define HABITRACK_MATCHES_CONTAINER_H

#include <filesystem>
#include <vector>

#include "habitrack/computeBehavior.h"
#include "habitrack/imageType.h"


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
    NN,
    Windowed
};

class MatchesContainer : public std::enable_shared_from_this<MatchesContainer>
{
public:
    MatchesContainer(std::shared_ptr<FeatureContainer> featureContainer,
        const std::filesystem::path& matchDir, MatchType type, std::size_t window = 0);

    void compute(std::size_t cacheSize, ComputeBehavior behavior = ComputeBehavior::Keep);

    /* std::vector<cv::KeyPoint> featureAt(std::size_t idx, ImageType imageType); */
    /* cv::Mat descriptorAt(std::size_t idx, ImageType imageType); */

    /* std::unique_ptr<FeatureCache> getFeatureCache(std::size_t maxChunkSize, */
    /*     ImageType imageType); */
    /* std::unique_ptr<DescriptorCache> getDescriptorCache(std::size_t maxChunkSize, */
    /*     ImageType imageType); */
private:
    /* FeatureType getTypeFromFile(const std::filesystem::path& file); */
    /* std::filesystem::path getFileName(std::size_t idx, detail::FtDesc ftDesc); */
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
    MatchType mType;
    std::size_t mWindow;
    bool mIsComputed;
};
} // namespace ht

#endif // HABITRACK_Matches_CONTAINER_H
