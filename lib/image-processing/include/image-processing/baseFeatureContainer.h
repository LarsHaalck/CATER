#ifndef HABITRACK_BASE_FEATURE_CONTAINER_H
#define HABITRACK_BASE_FEATURE_CONTAINER_H

#include "image-processing/baseImageContainer.h"
#include "image-processing/descriptorCache.h"
#include "image-processing/featureCache.h"
#include "image-processing/pairwiseDescriptorCache.h"
#include "image-processing/pairwiseFeatureCache.h"
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

    virtual FeatureCache getFeatureCache(
        std::size_t maxChunkSize, const size_t_vec& ids = size_t_vec()) const = 0;
    virtual DescriptorCache getDescriptorCache(
        std::size_t maxChunkSize, const size_t_vec& ids = size_t_vec()) const = 0;
    virtual PairwiseDescriptorCache getPairwiseDescriptorCache(std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const = 0;
    virtual PairwiseFeatureCache getPairwiseFeatureCache(std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const = 0;

    virtual cv::Size getImageSize() const = 0;
    virtual FeatureType getFeatureType() const = 0;
    virtual std::size_t size() const = 0;
};
} // namespace ht

#endif // HABITRACK_BASE_FEATURE_CONTAINER_H
