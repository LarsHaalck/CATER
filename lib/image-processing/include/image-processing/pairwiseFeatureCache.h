#ifndef HABITRACK_PAIRWISE_FEATURE_CACHE_H
#define HABITRACK_PAIRWISE_FEATURE_CACHE_H

#include "cache/basePairwiseCache.h"
#include <memory>
#include <opencv2/core.hpp>
#include <unordered_map>
#include <vector>

namespace ht
{
class BaseFeatureContainer;
}

namespace ht
{
class PairwiseFeatureCache : public BasePairwiseCache
{
public:
    PairwiseFeatureCache(const BaseFeatureContainer& container, std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs);

    std::unordered_map<std::size_t, std::vector<cv::KeyPoint>> getChunk(std::size_t idx);

private:
    const BaseFeatureContainer& mContainer;
};
}
#endif // HABITRACK_PAIRWISE_FEATURE_CACHE_H
