#ifndef CATER_PAIRWISE_DESCRIPTOR_CACHE_H
#define CATER_PAIRWISE_DESCRIPTOR_CACHE_H

#include <cater/cache/basePairwiseCache.h>
#include <memory>
#include <opencv2/core.hpp>
#include <unordered_map>

namespace ct
{
class BaseFeatureContainer;
}

namespace ct
{
class PairwiseDescriptorCache : public BasePairwiseCache
{
public:
    PairwiseDescriptorCache(const BaseFeatureContainer& container, std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs);

    std::unordered_map<std::size_t, cv::Mat> getChunk(std::size_t idx);

private:
    const BaseFeatureContainer& mContainer;
};
}
#endif // CATER_PAIRWISE_DESCRIPTOR_CACHE_H
