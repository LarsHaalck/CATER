#ifndef CATER_BASE_FEATURE_CONTAINER_H
#define CATER_BASE_FEATURE_CONTAINER_H

#include <cater/image-processing/baseImageContainer.h>
#include <cater/image-processing/descriptorCache.h>
#include <cater/image-processing/featureCache.h>
#include <cater/image-processing/pairwiseDescriptorCache.h>
#include <cater/image-processing/pairwiseFeatureCache.h>
#include <opencv2/features2d.hpp>
#include <vector>

namespace ct
{
enum class FeatureType
{
    ORB,
    SIFT,
    SuperPoint
};

std::ostream& operator<<(std::ostream& os, const FeatureType& type);

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
} // namespace ct

#endif // CATER_BASE_FEATURE_CONTAINER_H
