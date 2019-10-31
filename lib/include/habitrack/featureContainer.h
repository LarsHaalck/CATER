#ifndef HABITRACK_FEATURE_CONTAINER_H
#define HABITRACK_FEATURE_CONTAINER_H

#include "habitrack/baseFeatureContainer.h"
#include "habitrack/computeBehavior.h"
#include "habitrack/descriptorCache.h"
#include "habitrack/featureCache.h"
#include "habitrack/imageContainer.h"
#include "habitrack/pairwiseDescriptorCache.h"
#include "habitrack/pairwiseFeatureCache.h"
#include <filesystem>
#include <opencv2/features2d.hpp>
#include <vector>

namespace ht
{
namespace detail
{
    enum class FtDesc : bool
    {
        Feature,
        Descriptor
    };
}

class FeatureContainer : public BaseFeatureContainer,
                         public std::enable_shared_from_this<FeatureContainer>
{
public:
    FeatureContainer(std::shared_ptr<ImageContainer> imgContainer,
        const std::filesystem::path& ftDir, FeatureType type, std::size_t numFeatures);

    void compute(std::size_t cacheSize, ComputeBehavior behavior = ComputeBehavior::Keep,
        const ImgIds& ids = ImgIds());

    std::vector<cv::KeyPoint> featureAt(std::size_t idx) const override;
    cv::Mat descriptorAt(std::size_t idx) const override;

    std::unique_ptr<FeatureCache> getFeatureCache(
        std::size_t maxChunkSize, const ImgIds& ids = ImgIds()) override;
    std::unique_ptr<DescriptorCache> getDescriptorCache(
        std::size_t maxChunkSize, const ImgIds& ids = ImgIds()) override;
    std::unique_ptr<PairwiseDescriptorCache> getPairwiseDescriptorCache(std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs) override;
    std::unique_ptr<PairwiseFeatureCache> getPairwiseFeatureCache(std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs) override;

    /* std::shared_ptr<ImageContainer> getImageContainer() const; */
    std::size_t getNumImgs() const override;
    cv::Size getImgSize() const override;
    FeatureType getFtType() const override;

private:
    FeatureType getTypeFromFile(const std::filesystem::path& file) const;
    std::filesystem::path getFileName(std::size_t idx, detail::FtDesc ftDesc) const;
    cv::Ptr<cv::Feature2D> getFtPtr() const;
    void writeChunk(std::pair<std::size_t, std::size_t> bounds,
        const std::vector<std::vector<cv::KeyPoint>>& fts, const std::vector<cv::Mat>& descs,
        const ImgIds& ids) const;
    void writeFts(const std::filesystem::path& file, const std::vector<cv::KeyPoint>& fts) const;
    void writeDescs(const std::filesystem::path& file, const cv::Mat& descs) const;

    bool isComputed(const ImgIds& ids) const;

private:
    std::shared_ptr<ImageContainer> mImgContainer;
    std::filesystem::path mFtDir;
    FeatureType mType;
    std::size_t mNumFeatures;
    std::size_t mNumImgs;
    cv::Size mImgSize;
};
} // namespace ht

#endif // HABITRACK_FEATURE_CONTAINER_H
