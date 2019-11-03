#ifndef HABITRACK_BASE_FEATURE_CONTAINER_H
#define HABITRACK_BASE_FEATURE_CONTAINER_H

#include "habitrack/baseImageContainer.h"
#include "habitrack/descriptorCache.h"
#include "habitrack/featureCache.h"
#include "habitrack/pairwiseDescriptorCache.h"
#include "habitrack/pairwiseFeatureCache.h"
#include <opencv2/features2d.hpp>
#include <vector>

namespace ht
{
enum class FeatureType
{
    ORB,
    SIFT
};

class BaseFeatureContainer
{
public:
    BaseFeatureContainer() = default;

    virtual std::vector<cv::KeyPoint> featureAt(std::size_t idx) const = 0;
    virtual cv::Mat descriptorAt(std::size_t idx) const = 0;

    virtual std::unique_ptr<FeatureCache> getFeatureCache(
        std::size_t maxChunkSize, const ImgIds& ids = ImgIds())
        = 0;
    virtual std::unique_ptr<DescriptorCache> getDescriptorCache(
        std::size_t maxChunkSize, const ImgIds& ids = ImgIds())
        = 0;
    virtual std::unique_ptr<PairwiseDescriptorCache> getPairwiseDescriptorCache(
        std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs)
        = 0;
    virtual std::unique_ptr<PairwiseFeatureCache> getPairwiseFeatureCache(
        std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs)
        = 0;

    virtual std::size_t getNumImgs() const = 0;
    virtual cv::Size getImgSize() const = 0;
    virtual FeatureType getFtType() const = 0;
};
} // namespace ht

#endif // HABITRACK_BASE_FEATURE_CONTAINER_H
