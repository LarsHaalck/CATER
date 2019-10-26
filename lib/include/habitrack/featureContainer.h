#ifndef HABITRACK_FEATURE_CONTAINER_H
#define HABITRACK_FEATURE_CONTAINER_H

#include "featureIO.h"
#include "habitrack/computeBehavior.h"
#include "habitrack/descriptorCache.h"
#include "habitrack/featureCache.h"
#include "habitrack/imageContainer.h"
#include "habitrack/pairwiseDescriptorCache.h"
#include "habitrack/pairwiseFeatureCache.h"
#include "matIO.h"
#include <filesystem>
#include <vector>

namespace ht
{
enum class FeatureType
{
    ORB,
    SIFT
};

namespace detail
{
    enum class FtDesc : bool
    {
        Feature,
        Descriptor
    };
}

class FeatureContainer : public std::enable_shared_from_this<FeatureContainer>
{
public:
    FeatureContainer(std::shared_ptr<ImageContainer> imgContainer,
        const std::filesystem::path& ftDir, FeatureType type, std::size_t numFeatures);

    void compute(std::size_t cacheSize, ComputeBehavior behavior = ComputeBehavior::Keep);

    std::vector<cv::KeyPoint> featureAt(std::size_t idx);
    cv::Mat descriptorAt(std::size_t idx);

    std::unique_ptr<FeatureCache> getFeatureCache(
        std::size_t maxChunkSize, const ImgIds& ids = ImgIds());
    std::unique_ptr<DescriptorCache> getDescriptorCache(
        std::size_t maxChunkSize, const ImgIds& ids = ImgIds());
    std::unique_ptr<PairwiseDescriptorCache> getPairwiseDescriptorCache(
        std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs);
    std::unique_ptr<PairwiseFeatureCache> getPairwiseFeatureCache(
        std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs);

    std::shared_ptr<ImageContainer> getImageContainer() const;
    std::filesystem::path getFtDir() const;
    std::size_t getNumImgs() const;
    cv::Size getImgSize() const;
    FeatureType getFtType() const;

private:
    FeatureType getTypeFromFile(const std::filesystem::path& file);
    std::filesystem::path getFileName(std::size_t idx, detail::FtDesc ftDesc);
    cv::Ptr<cv::Feature2D> getFtPtr();
    void writeChunk(std::pair<std::size_t, std::size_t> bounds,
        const std::vector<std::vector<cv::KeyPoint>>& fts, const std::vector<cv::Mat>& descs);
    void writeFts(const std::filesystem::path& file, const std::vector<cv::KeyPoint>& fts);
    void writeDescs(const std::filesystem::path& file, const cv::Mat& descs);

private:
    std::shared_ptr<ImageContainer> mImgContainer;
    std::filesystem::path mFtDir;
    FeatureType mType;
    std::size_t mNumFeatures;
    std::size_t mNumImgs;
    cv::Size mImgSize;
    bool mIsComputed;
};
} // namespace ht

#endif // HABITRACK_FEATURE_CONTAINER_H
