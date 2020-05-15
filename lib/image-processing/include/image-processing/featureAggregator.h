#ifndef HABITRACK_FEATURE_AGGREGATOR_H
#define HABITRACK_FEATURE_AGGREGATOR_H

#include "image-processing/baseFeatureContainer.h"
#include "image-processing/featureContainer.h"

namespace ht
{
class FeatureAggregator : public BaseFeatureContainer,
                          public std::enable_shared_from_this<FeatureAggregator>
{
public:
    FeatureAggregator(const std::vector<std::shared_ptr<FeatureContainer>>& mFtContainers);
    FeatureAggregator(std::shared_ptr<FeatureContainer> ftContainer);

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

    std::size_t getNumImgs() const override;
    cv::Size getImgSize() const override;
    FeatureType getFtType() const override;

private:
    std::pair<std::size_t, std::vector<std::size_t>> sumNumImgs(
        const std::vector<std::shared_ptr<FeatureContainer>>& ftContainers) const;

private:
    std::vector<std::shared_ptr<FeatureContainer>> mFtContainers;
    std::size_t mNumImgs;
    std::vector<std::size_t> mNumImgsVec;
};
} // namespace ht
#endif // HABITRACK_FEATURE_AGGREGATOR_H
