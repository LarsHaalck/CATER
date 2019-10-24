#ifndef HABITRACK_PAIRWISE_DESCRIPTOR_CACHE_H
#define HABITRACK_PAIRWISE_DESCRIPTOR_CACHE_H

#include <memory>
#include <unordered_map>

#include <opencv2/core.hpp>

#include "basePairwiseCache.h"

namespace ht
{
class FeatureContainer;
}

namespace ht
{
class PairwiseDescriptorCache : public BasePairwiseCache
{
public:
    PairwiseDescriptorCache(std::shared_ptr<FeatureContainer> container, std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs);

    std::unordered_map<std::size_t, cv::Mat> getChunk(std::size_t idx);

private:
    std::shared_ptr<FeatureContainer> mContainer;
};
}
#endif // HABITRACK_PAIRWISE_DESCRIPTOR_CACHE_H
