#ifndef HABITRACK_FEATURE_AGGREGATOR_H
#define HABITRACK_FEATURE_AGGREGATOR_H

#include <habitrack/image-processing/baseFeatureContainer.h>
#include <habitrack/image-processing/features.h>

namespace ht
{
class FeatureAggregator : public BaseFeatureContainer
{
public:
    FeatureAggregator(const std::vector<Features>& mFtContainers);
    FeatureAggregator(const Features& ftContainer);

    std::vector<cv::KeyPoint> featureAt(std::size_t idx) const override;
    cv::Mat descriptorAt(std::size_t idx) const override;

    FeatureCache getFeatureCache(
        std::size_t maxChunkSize, const size_t_vec& ids = size_t_vec()) const override;
    DescriptorCache getDescriptorCache(
        std::size_t maxChunkSize, const size_t_vec& ids = size_t_vec()) const override;
    PairwiseDescriptorCache getPairwiseDescriptorCache(std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const override;
    PairwiseFeatureCache getPairwiseFeatureCache(std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const override;

    cv::Size getImageSize() const override;
    FeatureType getFeatureType() const override;
    std::size_t size() const override;

    std::vector<std::size_t> getBlockList() const;

private:
    std::pair<std::size_t, std::vector<std::size_t>> sumNumImgs(
        const std::vector<Features>& ftContainers) const;

private:
    const std::vector<Features>& mFtContainers;
    std::size_t mNumImgs;
    std::vector<std::size_t> mNumImgsVec;
};
} // namespace ht
#endif // HABITRACK_FEATURE_AGGREGATOR_H
