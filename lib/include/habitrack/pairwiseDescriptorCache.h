#ifndef HABITRACK_PAIRWISE_DESCRIPTOR_CACHE_H
#define HABITRACK_PAIRWISE_DESCRIPTOR_CACHE_H

#include <memory>
#include <unordered_map>

#include <opencv2/core.hpp>

#include "habitrack/basePairwiseCache.h"

namespace ht
{
class BaseFeatureContainer;
}

namespace ht
{
class PairwiseDescriptorCache : public BasePairwiseCache
{
public:
    PairwiseDescriptorCache(const BaseFeatureContainer& container,
        std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs);

    std::unordered_map<std::size_t, cv::Mat> getChunk(std::size_t idx);

private:
    const BaseFeatureContainer& mContainer;
};
}
#endif // HABITRACK_PAIRWISE_DESCRIPTOR_CACHE_H
