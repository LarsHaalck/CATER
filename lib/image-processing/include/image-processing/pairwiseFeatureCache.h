#ifndef HABITRACK_PAIRWISE_FEATURE_CACHE_H
#define HABITRACK_PAIRWISE_FEATURE_CACHE_H

#include <memory>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

#include "cache/basePairwiseCache.h"

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
